"""The objectives the tuner can minimise.

Two are available, and they want opposite measurement protocols:

  * thd()  -- total harmonic distortion of the output voltage. A steady-state
    quantity: the loop must be given time to settle before the window opens,
    and the integrators should NOT be reset between candidates or every
    evaluation pays the rebuild transient.
  * itae() -- integral of time-weighted absolute error. A transient quantity:
    it only means anything if t=0 is the start of one, so the integrators are
    reset as the candidate is applied and the window opens right there.

Picking the wrong protocol for the objective is what makes a tuning run measure
something other than what it is trying to minimise, so TunerWindow switches both
together.

ITAE:

    J(Kp, Ki) = integral over [0, Tm] of  t * |e(t)| dt

ITAE -- integral of time-weighted absolute error. The t factor is what makes it
a settling criterion rather than an accuracy one: error early in the window is
nearly free, error still present at the end of it is expensive. That only means
anything if t = 0 coincides with the start of a transient, which is why the
tuner resets the controller's integrators at the instant it applies a candidate
and starts the clock there (see TunerWindow._apply_candidate).

e(t) is the HCA error signal the firmware already streams (r_t minus the
normalised measurement, per unit), so J has units of per-unit-seconds squared.
Only its ordering matters to the optimizer, not its scale.
"""

import numpy as np

# np.trapz was removed in NumPy 2.0 in favour of np.trapezoid; the GUI has to
# run on both.
_trapezoid = getattr(np, "trapezoid", None) or np.trapz


def itae(t: np.ndarray, error: np.ndarray) -> float:
    """Integral of t*|e(t)| over the measurement window.

    Args:
        t: sample times in seconds. Rebased here so the window starts at t=0 --
           passing device timestamps directly is fine.
        error: the error signal at those times.

    Trapezoidal rather than a rectangle sum: the stream can drop frames, which
    leaves uneven gaps, and the trapezoid stays right across them.
    """
    if t.size < 2:
        return float("inf")

    t = t - t[0]
    return float(_trapezoid(t * np.abs(error), t))


def thd(voltage: np.ndarray, sample_rate: float, fundamental_hz: float = 50.0,
        max_harmonic: int = 49) -> float:
    """Total harmonic distortion of the voltage, as a fraction (0.05 = 5%).

        THD = sqrt(sum of V_h^2 for h = 2..max_harmonic) / V_1

    The window is trimmed to a whole number of fundamental periods before the
    transform, keeping the *newest* ones. That is what puts every harmonic
    exactly on a bin: with 5kHz samples and a 50Hz fundamental there are exactly
    100 samples per period, so harmonic h lands on bin h*cycles and no window
    function or interpolation is needed. A partial period would smear the
    fundamental across neighbouring bins and inflate the result -- which the
    optimizer would then happily minimise by changing nothing real.

    Trimming from the end rather than the start matters when the caller hands
    over slightly more than it asked for (samples arrive in batches, so a 30ms
    request can come back as 50ms): the newest samples are the ones the caller
    meant by "the last N ms".

    A single cycle is enough to be exact -- harmonic h simply lands on bin h --
    so short windows are supported. They are exact but not averaged: one cycle
    is a snapshot of whatever the waveform was doing in those 20ms, and its
    spread across repeats is correspondingly wide.

    DC is excluded (the sensor's offset is not distortion), and harmonics are
    counted up to max_harmonic or Nyquist, whichever comes first.
    """
    samples_per_cycle = sample_rate / fundamental_hz
    cycles = int(voltage.size // samples_per_cycle)
    if cycles < 1:
        return float("inf")

    trimmed = voltage[-int(round(cycles * samples_per_cycle)):]
    magnitude = np.abs(np.fft.rfft(trimmed))

    fundamental_bin = cycles
    if fundamental_bin >= magnitude.size or magnitude[fundamental_bin] <= 0.0:
        return float("inf")

    harmonic_bins = range(2 * cycles, magnitude.size, cycles)
    harmonics = np.array([magnitude[b] for b in harmonic_bins
                          if b // cycles <= max_harmonic])
    if harmonics.size == 0:
        return float("inf")

    return float(np.sqrt(np.sum(harmonics ** 2)) / magnitude[fundamental_bin])
