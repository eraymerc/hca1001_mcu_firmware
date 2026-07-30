/**
 * @file hca_lib.c
 *
 * @brief
 * High-performance C implementation of the Harmonic Control Array (HCA)
 * controller using parallel complex PI controllers.
 *
 * @details
 * This library implements the Harmonic Control Array (HCA) control structure
 * using Direct Digital Synthesis (DDS) and LUT-based trigonometric evaluation
 * for deterministic real-time execution.
 *
 * The implementation focuses on clarity, numerical robustness, and suitability
 * for embedded real-time systems, while remaining portable to desktop platforms.
 *
 * @author
 * Eray Mercan
 *
 * @par Algorithm attribution
 * The control algorithm implemented in this file is based on:
 *
 * Murat Doğruel and Hasan Hüseyin Çelik,
 * "Harmonic Control Arrays Method With a Real Time Application to Periodic Position Control,"
 * IEEE Transactions on Control Systems Technology,
 * vol. 19, Issue: 3, May 2011.
 * DOI: 10.1109/TCST.2010.2048110
 *
 * ---
 * License:
 *
 * This project is licensed under the MIT License.
 * See the LICENSE file in the repository root for full license text.
 *
 */

#include "hca_lib.h"
#include <string.h>
#include <stdint.h>
#include <math.h>


// ============================================================
// Global variables
// ============================================================

/**
 * @brief Sine lookup table used for DDS-based phasor generation
 *
 * The table covers one full electrical period [0, 2π).
 */
float HCA_sin_table[LUT_SIZE];

/**
 * @brief LUT initialization flag
 *
 * Ensures the lookup table is generated only once.
 */
static uint8_t is_lut_initialized = 0;

// ============================================================
// Internal complex math helper functions
// ============================================================

/**
 * @brief Add two complex numbers
 */
static inline Complex_t Complex_Add(Complex_t a, Complex_t b) {
    Complex_t res;
    res.real = a.real + b.real;
    res.imag = a.imag + b.imag;
    return res;
}

/**
 * @brief Multiply two complex numbers
 *
 * (a_r + j a_i) * (b_r + j b_i)
 */
static inline Complex_t Complex_Mult(Complex_t a, Complex_t b) {
    Complex_t res;
    res.real = (a.real * b.real) - (a.imag * b.imag);
    res.imag = (a.real * b.imag) + (a.imag * b.real);
    return res;
}

/**
 * @brief Scale a complex number by a real scalar
 */
static inline Complex_t Complex_Scale(Complex_t a, float scalar) {
    Complex_t res;
    res.real = a.real * scalar;
    res.imag = a.imag * scalar;
    return res;
}

// ============================================================
// Lookup table generation
// ============================================================

void HCA_CreateLUTs(void) {

    if (is_lut_initialized) return;

    // Divide the full circle into equally spaced samples
    float step = (2.0f * M_PI) / (float)LUT_SIZE;

    for (int i = 0; i < LUT_SIZE; i++) {
        // Angle in radians corresponding to LUT index
        float angle = (float)i * step;

        // Store sine value
        HCA_sin_table[i] = sinf(angle);
    }

    is_lut_initialized = 1;
}

// ============================================================
// DDS-based trigonometric helpers
// ============================================================

void HCA_getPhasor(uint32_t phase, Complex_t* complex) {
    // 1. Reduce 32-bit phase accumulator to LUT index
    uint32_t idx_sin = phase >> (32 - LUT_BITS);

    // 2. Cosine is sine shifted by +90 degrees
    uint32_t idx_cos = (idx_sin + (LUT_SIZE >> 2)) & LUT_MASK;

    complex->real = HCA_sin_table[idx_cos]; // cos(theta)
    complex->imag = HCA_sin_table[idx_sin]; // sin(theta)
}

float HCA_fastSin(uint32_t phase) {
    uint32_t idx_sin = phase >> (32 - LUT_BITS);
    return HCA_sin_table[idx_sin];
}

float HCA_fastCos(uint32_t phase) {
    uint32_t idx_sin = phase >> (32 - LUT_BITS);
    uint32_t idx_cos = (idx_sin + (LUT_SIZE >> 2)) & LUT_MASK;
    return HCA_sin_table[idx_cos];
}

// ============================================================
// HCA core functions
// ============================================================

void HCA_Init(HCA_Handle_t *hca,
              float fund_freq,
              float switch_freq,
              uint8_t oversample_ratio,
              float out_limit) {

    // Ensure sine LUT is ready
    HCA_CreateLUTs();

    if (oversample_ratio < 1) oversample_ratio = 1;

    hca->fundamental_freq = fund_freq;

    // Effective control loop frequency
    float loop_freq = switch_freq * (float)oversample_ratio;
    hca->sampling_time = 1.0f / loop_freq;

    hca->active_channel_count = 0;
    hca->output_limit = out_limit;
    
    // Disperser
    float N = loop_freq / fund_freq; // N = T/T_s = f_s / f 
    hca->window_len = N;
    hca->inv_window_len = 1.0f / (float)N;
    hca->buf_head = 0;

    // Clear circular buffer
    memset(hca->buffer, 0, sizeof(hca->buffer));

    // Reset all channels
    for (int i = 0; i < MAX_HARMONICS; i++) {

        hca->channels[i].harmonic_order = 0;

        hca->channels[i].current_val.real = 0.0f;
        hca->channels[i].current_val.imag = 0.0f;

        // PI integrator reset
        hca->channels[i].integrator_state.real = 0.0f;
        hca->channels[i].integrator_state.imag = 0.0f;
    }
}

void HCA_Add_Channel(HCA_Handle_t *hca,
                     uint8_t order,
                     Complex_t kp,
                     Complex_t ki) {

    if (hca->active_channel_count >= MAX_HARMONICS) return;

    int idx = hca->active_channel_count;
    HCA_Channel_t *ch = &hca->channels[idx];

    ch->harmonic_order = order;
    ch->Kp = kp;
    ch->Ki = ki;

    // Harmonic frequency = order * fundamental
    float harmonic_freq = hca->fundamental_freq * (float)order;

    // DDS phase increment: phase += omega * Ts * 2^32
    // For 0th harmonic (DC), harmonic_freq is 0, so angle_step becomes 0 (constant phase).
    ch->angle_step = (uint32_t)((harmonic_freq * hca->sampling_time)
                                * 4294967296.0f);
    ch->current_angle = 0;

    // Disperser window length calculation
    // SPECIAL CASE FOR DC (Order 0):
    // For DC, we cannot use 1/f since f=0.
    // The DC component is the average value over one FUNDAMENTAL period.
    // Therefore, for order 0, we use the fundamental frequency to calculate window size.
    float freq_for_window = (order == 0) ? hca->fundamental_freq : harmonic_freq;

    uint16_t Ns = (uint16_t)(1.0f /
                   (freq_for_window * hca->sampling_time));

    if (Ns > MAX_WINDOW_SIZE) Ns = MAX_WINDOW_SIZE;

    // Reset PI integrator
    ch->integrator_state.real = 0.0f;
    ch->integrator_state.imag = 0.0f;

    hca->active_channel_count++;
}

float HCA_Process(HCA_Handle_t *hca, float input_signal) {

    if (!is_lut_initialized) return 0.0f;

    float total_u_t = 0.0f;

    // Disperser - harmonic step
    // e[n]-e[n-N]*(1/N)
    int32_t read_index = (hca->buf_head - hca->window_len) & WINDOW_MASK;
    float delayed_val = hca->buffer[read_index];
    float harmonic_step = (input_signal - delayed_val)*hca->inv_window_len;

    // Update circular buffer
    hca->buffer[hca->buf_head] = input_signal;
    hca->buf_head = (hca->buf_head + 1) & WINDOW_MASK;

    for (int i = 0; i < hca->active_channel_count; i++) {

        HCA_Channel_t *ch = &hca->channels[i];

        // ----------------------------------------------------
        // DDS phase update
        // ----------------------------------------------------
        ch->current_angle += ch->angle_step;

        // Phase-compensated rotating phasor
        Complex_t rot_pos = {0.0f, 0.0f};
        HCA_getPhasor(
            ch->current_angle +
            ch->angle_step * HCA_DELAY_COMPENSATION_MULTIPLIER,
            &rot_pos
        );

        // ====================================================
        // DISPERSER
        // ====================================================
        // Input -> Step -> Modulation

        // 1. Modulation: x * e^{-jωt}
        Complex_t modulated_val;
        modulated_val.real = harmonic_step * rot_pos.real;
        modulated_val.imag = harmonic_step * (-rot_pos.imag);

        ch->current_val = Complex_Add(ch->current_val, modulated_val);

        // ====================================================
        // COMPLEX PI CONTROLLER
        // ====================================================
        // error_dq -> PI -> output_dq

        Complex_t error_dq = ch->current_val;

        // Proportional term
        Complex_t prop_term = Complex_Mult(ch->Kp, error_dq);

        // Integral term
        Complex_t ki_term = Complex_Mult(ch->Ki, error_dq);
        Complex_t ki_step =
            Complex_Scale(ki_term, hca->sampling_time);

        ch->integrator_state =
            Complex_Add(ch->integrator_state, ki_step);

        // Anti-windup saturation
        float limit = hca->output_limit;
        if (ch->integrator_state.real > limit)
            ch->integrator_state.real = limit;
        if (ch->integrator_state.real < -limit)
            ch->integrator_state.real = -limit;
        if (ch->integrator_state.imag > limit)
            ch->integrator_state.imag = limit;
        if (ch->integrator_state.imag < -limit)
            ch->integrator_state.imag = -limit;

        Complex_t output_dq =
            Complex_Add(prop_term, ch->integrator_state);

        // ====================================================
        // ASSEMBLER
        // ====================================================
        // Inverse modulation and harmonic reconstruction

        // 1. Rotate back: output_dq * e^{+jωt}
        Complex_t rotated_val =
            Complex_Mult(output_dq, rot_pos);

        // 2. Take real part and scale
        // SPECIAL CASE FOR DC (Order 0):
        // For AC harmonics, the complex phasor representation splits amplitude into 
        // two counter-rotating vectors (Euler's formula), requiring a gain of 2.0 
        // to reconstruct the original amplitude.
        // For DC (Order 0), there is no splitting (cos(0)=1), so the gain must be 1.0.
        float current_gain = (ch->harmonic_order == 0) ? 1.0f : 2.0f;
        
        float channel_out =
            rotated_val.real * current_gain;

        // 3. Sum all harmonic channels
        total_u_t += channel_out;
    }

    // Output saturation
    if (total_u_t > hca->output_limit)
        total_u_t = hca->output_limit;
    if (total_u_t < -hca->output_limit)
        total_u_t = -hca->output_limit;

    return total_u_t;
}

void HCA_UpdateChannel(HCA_Handle_t *hca,
                       uint8_t       order,
                       Complex_t     kp,
                       Complex_t     ki)
{
    for (int i = 0; i < hca->active_channel_count; i++) {
        if (hca->channels[i].harmonic_order == order) {
            /*
             * Write Kp first, then Ki.  The ISR may read between these two
             * writes — see header note.  For tuning purposes this is fine.
             */
            hca->channels[i].Kp = kp;
            hca->channels[i].Ki = ki;
            return;
        }
    }
    /* order not found — silently ignore */
}

void HCA_reset_accumulators(HCA_Handle_t* hca) {
    if (!hca) return;

    hca->buf_head = 0;
        
    // Clear the entire sliding window buffer
    memset(hca->buffer, 0, sizeof(float) * MAX_WINDOW_SIZE);

    for (uint8_t i = 0; i < hca->active_channel_count; i++) {
        HCA_Channel_t* ch = &hca->channels[i];
        
        // 1. Reset Integrators
        ch->integrator_state.real = 0.0f;
        ch->integrator_state.imag = 0.0f;
        
        // 2. Reset Disperser State (Full Clear)
        ch->current_val.real = 0.0f;
        ch->current_val.imag = 0.0f;

        //ch->current_angle = (uint32_t)0;
    }
}

void HCA_reset_accumulators_fast(HCA_Handle_t* hca) {
    if (!hca) return;
    hca->buf_head = 0;

    for (uint8_t i = 0; i < hca->active_channel_count; i++) {
        HCA_Channel_t* ch = &hca->channels[i];
        
        // 1. Reset Integrators
        ch->integrator_state.real = 0.0f;
        ch->integrator_state.imag = 0.0f;

        // 2. Reset Disperser Sum and Head
        // We reset the sum and head, but leave the buffer data as-is.
        // It will be overwritten as the head advances.
        ch->current_val.real = 0.0f;
        ch->current_val.imag = 0.0f;

        //ch->current_angle = (uint32_t)0;

    }
}
