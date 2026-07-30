/**
 * @file hca_lib.h
 * @author Eray Mercan
 *
 * @brief High-performance C implementation of the Harmonic Control Arrays (HCA) method.
 *
 * This library implements the Harmonic Control Arrays (HCA) controller using
 * parallel complex PI controllers operating in rotating reference frames.
 * The implementation is optimized for real-time embedded systems using
 * Direct Digital Synthesis (DDS) and lookup-table-based phasor generation.
 *
 * ---
 * Algorithm attribution:
 *
 * The Harmonic Control Arrays (HCA) method implemented in this library is based on:
 *
 * Murat Doğruel and Hasan Hüseyin Çelik,
 * "Harmonic Control Arrays Method With a Real Time Application to Periodic Position Control,"
 * IEEE Transactions on Control Systems Technology, vol. 19, Issue: 3, May 2011.
 * DOI: 10.1109/TCST.2010.2048110
 *
 * ---
 * License:
 *
 * This project is licensed under the MIT License.
 * See the LICENSE file in the repository root for full license text.
 *
 */

#ifndef INC_HCA_LIB_H_
#define INC_HCA_LIB_H_

#include <stdint.h>
#include <math.h>

/* ============================================================================
 * CONFIGURATION
 * ============================================================================
 */

/** @brief Maximum number of harmonic channels processed in parallel */
#define MAX_HARMONICS 5

/** @brief Lookup table resolution in bits */
#define LUT_BITS 12

/** @brief Lookup table size (2^LUT_BITS) */
#define LUT_SIZE  (1 << LUT_BITS)

/** @brief Mask for LUT index wrapping */
#define LUT_MASK  (LUT_SIZE - 1)

/**
 * @brief Maximum disperser window size.
 *
 * This buffer implements the z^-d delay line used in the disperser block.
 * Must be a power of two to enable efficient modulo indexing.
 */
#define MAX_WINDOW_SIZE 4096

/** @brief Mask for disperser circular buffer indexing */
#define WINDOW_MASK     (MAX_WINDOW_SIZE - 1)

/**
 * @brief Phase lead compensation multiplier.
 *
 * Used to compensate computational delay by advancing the phasor angle
 * during reconstruction (assembler stage).
 */
#define HCA_DELAY_COMPENSATION_MULTIPLIER 0.0f


/* ============================================================================
 * TYPE DEFINITIONS
 * ============================================================================
 */

/**
 * @brief Complex number representation.
 *
 * Used throughout the HCA implementation to represent signals
 * in rotating reference frames (dq-domain).
 */
typedef struct {
    float real; /**< Real component */
    float imag; /**< Imaginary component */
} Complex_t;


/**
 * @brief Per-harmonic HCA channel structure.
 *
 * Each channel corresponds to one harmonic order (k·ω₀) and implements:
 * - DDS-based phase tracking
 * - Disperser (modulation + sliding window integration)
 * - Complex PI controller
 * * @note For 0th harmonic (DC):
 * - Phase is constant (0 rad).
 * - Disperser window length is equal to the fundamental period (to average out AC).
 * - Assembler gain is 1.0 (instead of 2.0).
 */
typedef struct {

    /** Harmonic order (0 = DC, 1 = fundamental, 3 = 3rd harmonic, etc.) */
    uint8_t harmonic_order;

    /** Current Value */
    Complex_t current_val;

    /* ---------------- CONTROL PARAMETERS ---------------- */

    /** Complex proportional gain (Kp) */
    Complex_t Kp;

    /** Complex integral gain (Ki) */
    Complex_t Ki;

    /** Integrator state of the complex PI controller */
    Complex_t integrator_state;

    /* ---------------- DDS PHASE TRACKING ---------------- */

    /** Current DDS phase accumulator (32-bit) */
    uint32_t current_angle;

    /** Phase increment per sample (Δθ = ω·Ts·2^32) */
    uint32_t angle_step;

} HCA_Channel_t;


/**
 * @brief Top-level HCA controller handle.
 *
 * Contains global configuration and an array of harmonic channels.
 * The user interacts with the controller exclusively through this handle.
 */
typedef struct {

    /** Array of harmonic control channels */
    HCA_Channel_t channels[MAX_HARMONICS];

    /** Number of active harmonic channels */
    uint8_t active_channel_count;

    /** Fundamental frequency of the periodic signal (Hz) */
    float fundamental_freq;

    /** Sampling period of the control loop (seconds) */
    float sampling_time;

    /** Output saturation limit (symmetric) */
    float output_limit;

    /* ---------------- DISPERSER BLOCK ----------------
     *
     * Implements:
     * - Modulation into rotating frame
     * - Sliding window integration
     * - (1 - z^-d) structure
     */
    
    /** Circular buffer implementing the z^-d delay line */
    float buffer[MAX_WINDOW_SIZE];

    /** Current write index of the circular buffer */
    uint16_t buf_head;

    /** Effective window length (number of samples) */
    uint16_t window_len;

    /**
     * Precomputed reciprocal of window length (1/Ns).
     * Division is significantly slower than multiplication on MCUs,
     * so this value is stored explicitly for performance reasons.
     */
    float inv_window_len;

} HCA_Handle_t;


/* ============================================================================
 * FUNCTION PROTOTYPES
 * ============================================================================
 */

/**
 * @brief Initialize the HCA controller.
 *
 * @param hca Pointer to HCA handle
 * @param fund_freq Fundamental frequency (Hz)
 * @param switch_freq Base switching / ISR frequency (Hz)
 * @param oversample_ratio Oversampling factor applied to switch_freq
 * @param out_limit Symmetric output saturation limit
 */
void HCA_Init(HCA_Handle_t *hca,
              float fund_freq,
              float switch_freq,
              uint8_t oversample_ratio,
              float out_limit);

/**
 * @brief Create sine lookup tables for DDS-based phasor generation.
 *
 * This function must be called once before using the controller.
 * It is automatically invoked inside HCA_Init().
 */
void HCA_CreateLUTs(void);

/**
 * @brief Generate a complex phasor from a DDS phase accumulator.
 *
 * @param phase 32-bit phase accumulator value
 * @param complex Pointer to output complex phasor (cos + j·sin)
 */
void HCA_getPhasor(uint32_t phase, Complex_t* complex);

/**
 * @brief Fast sine evaluation using DDS LUT.
 *
 * @param phase 32-bit DDS phase
 * @return Sine of phase angle
 */
float HCA_fastSin(uint32_t phase);

/**
 * @brief Fast cosine evaluation using DDS LUT.
 *
 * @param phase 32-bit DDS phase
 * @return Cosine of phase angle
 */
float HCA_fastCos(uint32_t phase);

/**
 * @brief Add a harmonic control channel to the HCA controller.
 *
 * @param hca Pointer to HCA handle
 * @param order Harmonic order (1 = fundamental, 3 = 3rd harmonic, etc.)
 * @param kp Complex proportional gain
 * @param ki Complex integral gain
 */
void HCA_Add_Channel(HCA_Handle_t *hca,
                     uint8_t order,
                     Complex_t kp,
                     Complex_t ki);

/**
 * @brief Execute one HCA control step.
 *
 * This function must be called at a fixed sampling rate.
 *
 * @param hca Pointer to HCA handle
 * @param input_signal Real-valued input signal
 * @return Control output (real-valued)
 */
float HCA_Process(HCA_Handle_t *hca, float input_signal);

/**
 * @brief Update Kp and Ki gains of an existing harmonic channel at runtime.
 *
 * Finds the channel whose harmonic_order matches @p order and overwrites
 * its Kp and Ki.  Both components are written together so the channel never
 * runs with a mixed old/new gain set.
 *
 * Thread-safety note: on Cortex-M4 each float write is a single STR
 * instruction (atomic for aligned 32-bit values).  Writing a Complex_t
 * (two floats) is NOT atomic.  The caller must ensure this runs from the
 * main loop, not from the ADC ISR.  At worst one HCA_Process call may see
 * the new Kp with the old Ki — which causes a single glitched output sample.
 * For gain-tuning purposes this is acceptable.
 *
 * @param hca   Pointer to HCA handle
 * @param order Harmonic order to update (must already exist via HCA_Add_Channel)
 * @param kp    New complex proportional gain
 * @param ki    New complex integral gain
 */
void HCA_UpdateChannel(HCA_Handle_t *hca,
                       uint8_t       order,
                       Complex_t     kp,
                       Complex_t     ki);

void HCA_reset_accumulators(HCA_Handle_t* hca);


void HCA_reset_accumulators_fast(HCA_Handle_t* hca);

#endif /* INC_HCA_LIB_H_ */