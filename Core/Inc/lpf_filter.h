/**
 * @file lpf_filter.h
 * @brief Fixed 1kHz-cutoff FIR low-pass filter for the ADC1 (voltage)
 *        measurement, applied ahead of the HCA control loop.
 *
 * 63-tap, Hamming-windowed sinc lowpass ("ideal" brick-wall response
 * approximated and made realizable via windowing), designed for:
 *   - Sample rate (Fs): 40000 Hz  (ADC1 conversion-complete ISR rate)
 *   - Cutoff (Fc):        1000 Hz (-6 dB point, standard windowed-sinc convention)
 *   - Group delay:      (63-1)/2 = 31 samples = 775us at 40kHz
 *
 * Unity DC gain (coefficients sum to 1.0), Type I linear-phase, so it
 * does not distort the passband waveform shape - only delays it and
 * attenuates content above the cutoff.
 *
 * Coefficients were generated with tools/generate_lpf_coeffs.py. Rerun
 * that script (edit FS/FC/N at the top) and paste the new LPF_Coeffs[]
 * body here if you need a different cutoff or steeper rolloff.
 */

#ifndef INC_LPF_FILTER_H_
#define INC_LPF_FILTER_H_

#define LPF_NUM_TAPS 63U

/**
 * @brief Push one new sample through the filter and return the filtered output.
 *
 * Stateful: maintains its own internal delay line. Call exactly once per
 * new raw sample (e.g. once per ADC1 conversion), in sample order - do
 * not call it more than once for the same sample, and do not call it
 * from more than one place for the same signal.
 *
 * @param sample Newest raw input sample
 * @return Filtered output (same units as the input)
 */
float LPF_Apply(float sample);

#endif /* INC_LPF_FILTER_H_ */
