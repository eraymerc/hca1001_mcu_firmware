/**
 * @file lpf_filter.c
 * @brief See lpf_filter.h. Direct-form FIR, shift-register delay line.
 *
 * 63 taps * 40000 samples/s = ~2.5M MACs/s, negligible load for the
 * Cortex-M4F FPU - simplicity/correctness of a plain shift register was
 * chosen over a circular-buffer index scheme.
 */

#include <stdint.h>
#include "lpf_filter.h"

/* Hamming-windowed sinc lowpass, Fs=40000Hz, Fc=1000Hz, unity DC gain.
 * LPF_Coeffs[0] is applied to the newest sample (see LPF_Apply). Generated
 * by tools/generate_lpf_coeffs.py. */
static const float LPF_Coeffs[LPF_NUM_TAPS] = {
    -8.226892918e-04f, -8.861011376e-04f, -9.829374772e-04f, -1.108331412e-03f,
    -1.249184263e-03f, -1.383957943e-03f, -1.483027386e-03f, -1.509604179e-03f,
    -1.421208766e-03f, -1.171634988e-03f, -7.133193393e-04f, 6.668645261e-19f,
    1.010470852e-03f, 2.353312359e-03f, 4.054155086e-03f, 6.126690621e-03f,
    8.570774186e-03f, 1.137110421e-02f, 1.449657588e-02f, 1.790037279e-02f,
    2.152082323e-02f, 2.528300843e-02f, 2.910106980e-02f, 3.288112483e-02f,
    3.652466720e-02f, 3.993229934e-02f, 4.300762552e-02f, 4.566112212e-02f,
    4.781380029e-02f, 4.940048379e-02f, 5.037254238e-02f, 5.069994653e-02f,
    5.037254238e-02f, 4.940048379e-02f, 4.781380029e-02f, 4.566112212e-02f,
    4.300762552e-02f, 3.993229934e-02f, 3.652466720e-02f, 3.288112483e-02f,
    2.910106980e-02f, 2.528300843e-02f, 2.152082323e-02f, 1.790037279e-02f,
    1.449657588e-02f, 1.137110421e-02f, 8.570774186e-03f, 6.126690621e-03f,
    4.054155086e-03f, 2.353312359e-03f, 1.010470852e-03f, 6.668645261e-19f,
    -7.133193393e-04f, -1.171634988e-03f, -1.421208766e-03f, -1.509604179e-03f,
    -1.483027386e-03f, -1.383957943e-03f, -1.249184263e-03f, -1.108331412e-03f,
    -9.829374772e-04f, -8.861011376e-04f, -8.226892918e-04f,
};

static float delay_line[LPF_NUM_TAPS] = {0};

float LPF_Apply(float sample)
{
    for (uint32_t i = LPF_NUM_TAPS - 1U; i > 0U; i--)
    {
        delay_line[i] = delay_line[i - 1U];
    }
    delay_line[0] = sample;

    float y = 0.0f;
    for (uint32_t i = 0; i < LPF_NUM_TAPS; i++)
    {
        y += LPF_Coeffs[i] * delay_line[i];
    }
    return y;
}
