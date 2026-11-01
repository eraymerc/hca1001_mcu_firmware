"""Fixed-length capture of the live stream, for the Export bar's Record button.

Pure Python, no Qt: the GUI feeds it the same AdcFrame batches it plots and
polls ``state`` afterwards. A capture is only ever handed back gap-free -- a
hole in seq means samples are missing, so the capture is thrown away and
started again rather than saved with a silent discontinuity in it.
"""

from protocol import STREAM_RATE_HZ

ARMING = "arming"        # waiting for the reference's rising zero crossing
RECORDING = "recording"
DONE = "done"
FAILED = "failed"

# Give up rather than retry forever on a link that keeps dropping frames.
MAX_RESTARTS = 5
# A reference stuck at zero (multiplier 0) never crosses; stop waiting after this.
ARM_TIMEOUT_SAMPLES = int(1.0 * STREAM_RATE_HZ)


class Recorder:
    """Collects n_samples contiguous samples.

    records holds (seq, timestamp_ms, t_s, voltage, reference, error) tuples,
    where t_s is seconds since the first recorded sample, derived from seq --
    exact at the stream rate, unlike the 1ms device timestamp.
    """

    def __init__(self, n_samples: int, align_to_reference: bool):
        self.n_samples = n_samples
        self.align = align_to_reference
        self.restarts = 0
        self.message = ""
        self.records = []
        self._prev = None        # last sample seen, for gap and crossing checks
        self._armed_for = 0
        self.state = ARMING if align_to_reference else RECORDING

    @property
    def progress(self) -> float:
        return len(self.records) / self.n_samples

    def feed(self, frames):
        for f in frames:
            if self.state in (DONE, FAILED):
                return
            prev, self._prev = self._prev, f

            if prev is not None and f.seq != prev.seq + 1:
                self._restart(f.seq - prev.seq - 1)
                if self.state == FAILED:
                    return
                prev = None  # a crossing cannot span the gap

            if self.state == ARMING:
                if prev is not None and prev.reference < 0.0 <= f.reference:
                    self.state = RECORDING
                else:
                    self._armed_for += 1
                    if self._armed_for > ARM_TIMEOUT_SAMPLES:
                        self.state = FAILED
                        self.message = (
                            "The reference did not cross zero within 1 s - is the "
                            "reference multiplier zero?"
                        )
                    continue

            seq0 = self.records[0][0] if self.records else f.seq
            self.records.append(
                (f.seq, f.timestamp_ms, (f.seq - seq0) / STREAM_RATE_HZ,
                 f.voltage, f.reference, f.error)
            )
            if len(self.records) >= self.n_samples:
                self.state = DONE

    def _restart(self, missing: int):
        self.restarts += 1
        self.records = []
        self._armed_for = 0
        if self.restarts > MAX_RESTARTS:
            self.state = FAILED
            self.message = (
                f"Samples kept going missing ({self.restarts} gaps in seq) - the "
                "link is not carrying the stream reliably."
            )
            return
        self.state = ARMING if self.align else RECORDING
        self.message = f"{missing} samples lost, capture restarted ({self.restarts}/{MAX_RESTARTS})"
