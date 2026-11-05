/**
 * @file pr_lib.c
 *
 * @brief Multi-resonant Proportional-Resonant (PR) controller with
 *        impulse-invariant, phase-compensated resonators.
 *
 * @details
 * See pr_lib.h for the transfer function, the discretization, the reasoning
 * behind the delta-form realization and the literature it follows
 * (Zmood & Holmes 2003; Teodorescu et al. 2006; Yepes et al. 2010).
 *
 * @author
 * Eray Mercan
 *
 * ---
 * License:
 *
 * This project is licensed under the MIT License.
 * See the LICENSE file in the repository root for full license text.
 */

#include "pr_lib.h"
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ============================================================
// Internal helpers
// ============================================================

/**
 * @brief Discrete coefficients of one resonator, computed in double.
 *
 * Done once per (order, Ki, phi) from the main loop, never from the ISR:
 * the Cortex-M4 FPU is single precision, so these are soft-float calls.
 */
typedef struct {
    float b0, db, eps, inv_sin2;
} PR_Coeffs_t;

static PR_Coeffs_t PR_ComputeCoeffs(const PR_Handle_t *pr,
                                    uint8_t order, float ki, float phi)
{
    PR_Coeffs_t c;
    const double ts = (double)pr->sampling_time;

    if (order == 0) {
        /* DC: R(s) = Ki/s, impulse invariant -> Ts*Ki / (1 - z^-1).
         * Handled as a plain integrator in PR_Process; phi has no meaning. */
        c.b0       = (float)(ts * (double)ki);
        c.db       = 0.0f;
        c.eps      = 0.0f;
        c.inv_sin2 = 0.0f;
        return c;
    }

    const double wts = 2.0 * M_PI * (double)order * (double)pr->fundamental_freq * ts;
    const double p   = (double)phi;
    const double s_half = sin(0.5 * wts);
    const double s_full = sin(wts);

    c.b0  = (float)(ts * (double)ki * cos(p));
    /* cos(phi) - cos(phi - wTs) = -2*sin(phi - wTs/2)*sin(wTs/2): no
     * cancellation even when wTs is tiny. */
    c.db  = (float)(ts * (double)ki * (-2.0 * sin(p - 0.5 * wts) * s_half));
    /* 2 - 2*cos(wTs), kept in full relative precision */
    c.eps = (float)(4.0 * s_half * s_half);
    c.inv_sin2 = (float)(1.0 / (s_full * s_full));
    return c;
}

static inline void Channel_ResetState(PR_Channel_t *ch)
{
    ch->d = 0.0f;
    ch->y = 0.0f;
}

// ============================================================
// PR core functions
// ============================================================

void PR_Init(PR_Handle_t *pr,
             float fund_freq,
             float switch_freq,
             uint8_t oversample_ratio,
             float kp,
             float out_limit)
{
    if (oversample_ratio < 1) oversample_ratio = 1;

    memset(pr, 0, sizeof(*pr));

    pr->fundamental_freq = fund_freq;
    pr->sampling_time    = 1.0f / (switch_freq * (float)oversample_ratio);
    pr->Kp               = kp;
    pr->output_limit     = fabsf(out_limit);
    pr->prev_error       = 0.0f;
    pr->active_channel_count = 0;
}

void PR_Add_Channel(PR_Handle_t *pr,
                    uint8_t order,
                    float ki,
                    float phi)
{
    if (pr->active_channel_count >= PR_MAX_HARMONICS) return;

    PR_Channel_t *ch = &pr->channels[pr->active_channel_count];
    PR_Coeffs_t c = PR_ComputeCoeffs(pr, order, ki, phi);

    ch->harmonic_order = order;
    ch->Ki       = ki;
    ch->phi      = phi;
    ch->b0       = c.b0;
    ch->db       = c.db;
    ch->eps      = c.eps;
    ch->inv_sin2 = c.inv_sin2;
    Channel_ResetState(ch);

    pr->active_channel_count++;
}

float PR_Process(PR_Handle_t *pr, float error)
{
    const float e_prev = pr->prev_error;
    const float de     = error - e_prev;
    const float limit  = pr->output_limit;
    const float limit2 = limit * limit;

    pr->prev_error = error;

    float total = pr->Kp * error;

    for (int i = 0; i < pr->active_channel_count; i++) {

        PR_Channel_t *ch = &pr->channels[i];

        if (ch->harmonic_order == 0) {
            // ------------------------------------------------
            // DC integrator: y += Ts*Ki*e, clamped (anti-windup)
            // ------------------------------------------------
            float y = ch->y + ch->b0 * error;
            if (y >  limit) y =  limit;
            if (y < -limit) y = -limit;
            ch->y = y;
            total += y;
            continue;
        }

        // ----------------------------------------------------
        // Numerator: u = Ts*Ki*(cos(phi)*e[n] - cos(phi - wTs)*e[n-1])
        // ----------------------------------------------------
        float u = ch->b0 * de + ch->db * e_prev;

        // ----------------------------------------------------
        // Denominator, delta form: y[n] = (2-eps)y[n-1] - y[n-2] + u[n]
        // ----------------------------------------------------
        float y_prev = ch->y;
        float d = ch->d - ch->eps * y_prev + u;
        float y = y_prev + d;

        // ----------------------------------------------------
        // Anti-windup: cap the resonator's amplitude at output_limit.
        // A^2 = (d^2 + eps*y*(y - d)) / sin^2(wTs). Scaling d and y together
        // shrinks the oscillation without touching its frequency or phase.
        // Only costs a sqrt on the samples where it actually clamps.
        // ----------------------------------------------------
        float amp2 = (d * d + ch->eps * y * (y - d)) * ch->inv_sin2;
        if (amp2 > limit2) {
            float k = sqrtf(limit2 / amp2);
            d *= k;
            y *= k;
        }

        ch->d = d;
        ch->y = y;

        total += y;
    }

    // Output saturation
    if (total >  limit) total =  limit;
    if (total < -limit) total = -limit;

    return total;
}

uint8_t PR_UpdateChannel(PR_Handle_t *pr,
                         uint8_t order,
                         float ki,
                         float phi)
{
    for (int i = 0; i < pr->active_channel_count; i++) {
        PR_Channel_t *ch = &pr->channels[i];
        if (ch->harmonic_order != order) continue;

        /* Slow part first, so the ISR-visible writes below are back to back. */
        PR_Coeffs_t c = PR_ComputeCoeffs(pr, order, ki, phi);

        ch->Ki       = ki;
        ch->phi      = phi;
        ch->b0       = c.b0;
        ch->db       = c.db;
        ch->eps      = c.eps;
        ch->inv_sin2 = c.inv_sin2;
        return 1;
    }
    return 0;
}

void PR_SetKp(PR_Handle_t *pr, float kp)
{
    pr->Kp = kp;
}

void PR_Reset(PR_Handle_t *pr)
{
    if (!pr) return;

    pr->prev_error = 0.0f;
    for (uint8_t i = 0; i < pr->active_channel_count; i++) {
        Channel_ResetState(&pr->channels[i]);
    }
}
