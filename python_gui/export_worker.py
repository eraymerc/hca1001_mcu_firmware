"""Background CSV export. Writing up to MAX_SESSION_SAMPLES (2,000,000) rows
synchronously on the GUI thread would freeze the UI for a noticeable time,
so this runs on its own QThread. Only touches plain Python data (no GUI
objects), so it's safe to run off the GUI thread.
"""

import csv

from PyQt5.QtCore import QThread, pyqtSignal


class CsvExportWorker(QThread):
    finished_ok = pyqtSignal(str, int)  # path, row count
    failed = pyqtSignal(str)

    def __init__(self, path, header, records, col_indices, parent=None):
        super().__init__(parent)
        self._path = path
        self._header = header
        # `records` should be a snapshot (e.g. list(session_records)) taken on
        # the GUI thread before starting this worker, so we never iterate a
        # list that the GUI thread is concurrently appending to.
        self._records = records
        self._col_indices = col_indices

    def run(self):
        try:
            with open(self._path, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(self._header)
                for rec in self._records:
                    writer.writerow([rec[0], rec[1]] + [rec[i] for i in self._col_indices])
        except OSError as exc:
            self.failed.emit(str(exc))
            return

        self.finished_ok.emit(self._path, len(self._records))
