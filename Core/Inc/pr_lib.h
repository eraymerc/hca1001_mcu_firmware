/**
 * @file pr_lib.h
 * @author Eray Mercan
 *
 * @brief Multi-resonant Proportional-Resonant (PR) controller.
 *
 *   G(s) = Kp + sum_h R_h(s)
 *
 *   R_h(s) = Ki_h * (s*cos(phi_h) - h*w0*sin(phi_h)) / (s^2 + (h*w0)^2)
 *
 * One ideal resonator per selected harmonic order h, in parallel with a
 * single broadband proportional gain. phi_h is a per-harmonic phase lead that
 * compensates the computation/PWM delay and plant phase at h*w0.
 *
 * Each resonator is discretized with the impulse-invariant method:
 *
 *   R_h(z) = Ts*Ki_h * (cos(phi_h) - z^-1*cos(phi_h - h*w0*Ts))
 *                    / (1 - 2*cos(h*w0*Ts)*z^-1 + z^-2)
 *
 * ---
 * Algorithm attribution:
 *
 * D. N. Zmood and D. G. Holmes, "Stationary frame current regulation of PWM
 * inverters with zero steady-state error," IEEE Trans. Power Electron.,
 * vol. 18, no. 3, pp. 814-822, 2003.
 *
 * R. Teodorescu, F. Blaabjerg, M. Liserre and P. C. Loh, "Proportional-
 * resonant controllers and filters for grid-connected voltage-source
 * converters," IEE Proc. Electr. Power Appl., vol. 153, no. 5, pp. 750-762,
 * 2006. DOI: 10.1049/ip-epa:20060008
 *
 * A. G. Yepes, F. D. Freijedo, J. Doval-Gandoy, O. Lopez, J. Malvar and
 * P. Fernandez-Comesana, "Effects of discretization methods on the
 * performance of resonant controllers," IEEE Trans. Power Electron.,
 * vol. 25, no. 7, pp. 1692-1712, 2010.
 *
 * ---
 * Relation to HCA (hca_lib.h):
 *
 * An HCA channel's integral path, without the disperser, is
 *   2*Re{ e^{jwt} * (Ki_hca/s) * [e(t) e^{-jwt}] }
 *     = 2|Ki_hca| * (s*cos(angle) - w*sin(angle)) / (s^2 + w^2),
 * i.e. exactly R_h(s) with
 *   Ki_h  = 2*|Ki_hca|,   phi_h = angle(Ki_hca).
 * Handy as a starting point for tuning, but the disperser (sliding DFT) adds
 * its own dynamics to HCA, so the two loops are not identical.
 *
 * ---
 * License:
 *
 * This project is licensed under the MIT License.
 * See the LICENSE file in the repository root for full license text.
 */

#ifndef INC_PR_LIB_H_
#define INC_PR_LIB_H_

#include <stdint.h>

/* ============================================================================
 * CONFIGURATION
 * ============================================================================
 */

/**
 * @brief Maximum number of resonators.
 *
 * Same idea as MAX_HARMONICS in hca_lib.h: slots are reserved statically, the
 * per-sample cost follows active_channel_count. A resonator is ~10 FLOPs, far
 * cheaper than an HCA channel.
 */
#define PR_MAX_HARMONICS 50

/** @brief Degrees-to-radians factor for phi_h given in degrees */
#define PR_DEG2RAD 0.017453292519943295f


/* ============================================================================
 * TYPE DEFINITIONS
 * ============================================================================
 */

/**
 * @brief One resonator, tuned to harmonic_order * fundamental.
 *
 * Realization -- why not a plain biquad:
 * At 40kHz the fundamental's pole angle w*Ts is only 7.85e-3 rad, so
 * 2*cos(w*Ts) = 1.99993836... In float that coefficient is quantized to
 * ~6e-8, which detunes the 50Hz resonator by ~0.02Hz and leaves it with a
 * finite gain at 50Hz -- non-zero steady-state error, the one thing PR is
 * there to remove. A direct-form biquad also amplifies state round-off by
 * ~1/(w*Ts)^2.
 *
 * So the denominator is run in delta form instead, with
 *   eps = 2 - 2*cos(w*Ts) = 4*sin^2(w*Ts/2)
 * computed in double at init (eps keeps full relative precision, unlike
 * 2*cos). With y the resonator output:
 *   d[n] = d[n-1] - eps*y[n-1] + u[n]
 *   y[n] = y[n-1] + d[n]
 * which is exactly y[n] = (2-eps)*y[n-1] - y[n-2] + u[n]. The poles stay on
 * the unit circle for any eps, and the frequency error drops to ~1e-6 Hz.
 *
 * The numerator is split the same way to avoid cancellation:
 *   u[n] = b0*(e[n] - e[n-1]) + db*e[n-1]
 *   b0 = Ts*Ki*cos(phi),  db = Ts*Ki*(cos(phi) - cos(phi - w*Ts))
 */
typedef struct {

    /** Harmonic order (0 = DC integrator, 1 = fundamental, 3, 5, ...) */
    uint8_t harmonic_order;

    /* ---------------- TUNING (as given by the user) ---------------- */

    /** Resonant gain Ki_h */
    float Ki;

    /** Phase lead phi_h, radians */
    float phi;

    /* ---------------- DISCRETE COEFFICIENTS (derived) ---------------- */

    float b0;   /**< Ts*Ki*cos(phi) */
    float db;   /**< Ts*Ki*(cos(phi) - cos(phi - w*Ts)) */
    float eps;  /**< 4*sin^2(w*Ts/2) */

    /**
     * 1 / sin^2(w*Ts), for the amplitude estimate used by anti-windup:
     * for a sinusoid of amplitude A the delta-form states satisfy
     *   d^2 + eps*y*(y - d) = A^2 * sin^2(w*Ts).
     */
    float inv_sin2;

    /* ---------------- STATE ---------------- */

    float d;    /**< First difference of the output, y[n] - y[n-1] */
    float y;    /**< Resonator output */

} PR_Channel_t;


/**
 * @brief Top-level PR controller handle.
 */
typedef struct {

    /** Resonator array */
    PR_Channel_t channels[PR_MAX_HARMONICS];

    /** Number of active resonators */
    uint8_t active_channel_count;

    /** Broadband proportional gain */
    float Kp;

    /** Fundamental frequency (Hz) */
    float fundamental_freq;

    /** Sampling period of the control loop (s) */
    float sampling_time;

    /**
     * Symmetric output saturation limit. Also the anti-windup ceiling on each
     * resonator's amplitude, mirroring HCA's integrator clamp.
     */
    float output_limit;

    /** e[n-1], shared by every resonator's numerator */
    float prev_error;

} PR_Handle_t;


/* ============================================================================
 * FUNCTION PROTOTYPES
 * ============================================================================
 */

/**
 * @brief Initialize the PR controller.
 *
 * Arguments mirror HCA_Init so the two controllers can be set up side by side.
 *
 * @param pr               Pointer to PR handle
 * @param fund_freq        Fundamental frequency (Hz)
 * @param switch_freq      Base switching / ISR frequency (Hz)
 * @param oversample_ratio Oversampling factor applied to switch_freq
 * @param kp               Broadband proportional gain
 * @param out_limit        Symmetric output saturation limit
 */
void PR_Init(PR_Handle_t *pr,
             float fund_freq,
             float switch_freq,
             uint8_t oversample_ratio,
             float kp,
             float out_limit);

/**
 * @brief Add a resonator at order * fundamental.
 *
 * Coefficients are computed here in double precision (soft-float, so keep
 * this out of the ISR).
 *
 * @param pr    Pointer to PR handle
 * @param order Harmonic order (0 = DC integrator Ki/s, phi ignored)
 * @param ki    Resonant gain Ki_h
 * @param phi   Phase lead phi_h in radians (use PR_DEG2RAD for degrees)
 */
void PR_Add_Channel(PR_Handle_t *pr,
                    uint8_t order,
                    float ki,
                    float phi);

/**
 * @brief Execute one PR control step. Call at the fixed sampling rate.
 *
 * @param pr    Pointer to PR handle
 * @param error Tracking error e[n] = r[n] - y[n]
 * @return Control output, saturated to +-output_limit
 */
float PR_Process(PR_Handle_t *pr, float error);

/**
 * @brief Retune an existing resonator at runtime.
 *
 * Same thread-safety caveat as HCA_UpdateChannel: call from the main loop.
 * The new coefficients are computed first and written back to back, so the
 * ISR can see at most one sample with a mixed old/new set. State is kept, so
 * the resonator carries on from its current oscillation.
 *
 * @return 1 if a resonator of that order exists, 0 otherwise
 */
uint8_t PR_UpdateChannel(PR_Handle_t *pr,
                         uint8_t order,
                         float ki,
                         float phi);

/** @brief Set the broadband proportional gain at runtime. */
void PR_SetKp(PR_Handle_t *pr, float kp);

/** @brief Zero every resonator's state (gains kept). */
void PR_Reset(PR_Handle_t *pr);

#endif /* INC_PR_LIB_H_ */
