# HCA1001 ADC Monitor (Go / Fyne GUI)

Go port of `python_gui/`. Dark-mode desktop app that connects to the
STM32G474RE over the **ST-LINK's virtual COM port** (LPUART1, 2,097,000 baud —
see `Core/Src/main.c`) and plots the single ADC1 voltage reading plus the HCA
control-loop error live. Just the one USB cable into the Nucleo's ST-LINK
connector — no external UART adapter needed.

Builds to a single binary with no Python runtime and no `pip install`.

## Wire protocol

One 19-byte frame per sample (sync + seq + timestamp + float voltage + float
error + checksum), decimated to 1 kHz from the 40 kHz control loop
(`ADC_STREAM_DECIMATION` in `main.c`). Full float precision, resyncs instantly
on any dropped/corrupted byte via the two sync bytes.

`protocol.go` must match `AdcStreamFrame_t` in `Core/Inc/main.h` byte-for-byte.
If you change the frame layout on the firmware side, update `FrameSize` and
`ParseFrame` here.

## Build

```
make            # host build
make run        # build and run
make test       # unit + pty integration tests
make linux      # dist/hca1001-monitor
make windows    # dist/hca1001-monitor.exe
make release    # both, stripped
make toolchain  # show which cross-compiler will be used
```

This `Makefile` is independent of the firmware `Makefile` in the project root.

Fyne is a cgo project, so a C toolchain and the usual OpenGL/X11 development
headers are required. On Arch/CachyOS those come from `base-devel`, `libgl`,
`libxcursor`, `libxrandr`, `libxinerama` and `libxi`.

### Windows cross-compilation

`make windows` needs a C compiler targeting Windows and picks one automatically,
preferring **zig**:

| | install | notes |
|---|---|---|
| zig (preferred) | `pacman -S zig` / `apt install zig` | ships its own Windows headers + CRT, nothing else to install |
| mingw-w64 | `pacman -S mingw-w64-gcc` / `apt install gcc-mingw-w64-x86-64` | |

Force one with `make windows TOOLCHAIN=zig` or `TOOLCHAIN=mingw`. Both are
tested and produce a GUI-subsystem `.exe` (no console window). zig needs
`CGO_LDFLAGS=-Wl,--subsystem,windows` to get that — its linker does not act on
Go's `-H windowsgui` — and the Makefile passes it automatically.

The first Windows build compiles all of Fyne's C for a new target and takes
several minutes. Go caches per-target, so later builds are quick; a slow first
run is expected, not a failure.

## Run

```
make run
```

On Linux your user needs read/write access to the serial device — usually
membership of the `uucp` (Arch) or `dialout` (Debian/Ubuntu) group.

## Usage

1. Flash the firmware, plug the Nucleo into USB (ST-LINK cable).
2. Click **Refresh** if your port isn't listed, select it in the **COM Port**
   dropdown, click **Connect**.
3. Click **Test Connection** to confirm you picked the right port — the
   firmware replies with an identification string and the status bar shows
   `Device identified: HCA1001_ADC_STREAM_V1`. If nothing happens within a
   second, you've likely got the wrong port or the firmware isn't flashed.
4. Click **Start Streaming**. Each row (Voltage, HCA Error) plots live at 1 kHz.
5. Click **FFT** next to any signal to open a live spectrum window for just that
   signal (2048-sample window, Hann-windowed, updates 4×/sec, Nyquist 500 Hz).
6. Use the **Export** checkbox on each row to choose which signals are included
   when exporting.
7. **Save Selected as CSV** — writes every captured sample (seq, timestamp_ms,
   and the checked signal columns) since you connected.
8. **Save Selected as PNG** — exports one PNG per checked signal's plot into a
   folder you choose.

## Trigger

Enable it, pick a source signal, edge and level, and every row freezes to the
most recent qualifying edge crossing instead of scrolling. All rows are sliced
from the *same* crossing even when their window lengths differ, so the two
plots stay time-aligned with each other. A dashed yellow line marks t=0.
"Waiting for trigger..." means no crossing has been seen that still leaves a
full window of samples after it.

## Design notes

- **Serial I/O is confined to one goroutine** (`serial.go`). Commands from the
  UI go through a channel rather than writing to the port directly, so a
  `Write()` that blocks on a USB hiccup can never stall the UI. Parsed frames
  go into a large buffered channel that a consumer goroutine drains into the
  `Monitor`.
- **Redraw is decoupled from arrival rate.** The UI ticks at 20 Hz
  (`guiUpdateInterval`) and takes a snapshot of the ring buffers. A slow redraw
  just means the next tick sees more samples — there is no per-sample backlog
  that can grow without bound.
- **Plots are rendered to a raster** (`plot.go`), not built from canvas
  objects. Fyne's canvas is a retained scene graph, so 30,000 samples as
  individual objects would be hopeless. The curve is min/max downsampled to one
  vertical segment per pixel column — the same trick pyqtgraph uses — making
  redraw cost depend on plot width rather than sample count. Antialiasing is
  deliberately off, as it was in the Python GUI.
- The live ring buffers hold `MaxLiveSamples` (30 s at 1 kHz); each row's
  **Window** field just selects how much of that trailing history it shows, so
  changing it never reallocates.
- The session buffer for CSV export is capped at 2,000,000 samples (~33 min at
  1 kHz) to bound RAM use. Save and reconnect to keep capturing past that.
- CSV export runs on its own goroutine over a snapshot of the session record,
  so writing 2M rows never freezes the UI.
- **Threading**: every background goroutine hands UI work to the main loop via
  `fyne.Do` (serial callbacks, the redraw ticker, the ping timeout, the CSV
  writer, the FFT refresh). `main.go` declares this with the `fyneDo` migration
  flag in `app.SetMetadata` — without that declaration Fyne prints a "*** This
  application has not been migrated to the fyne.Do threading model ***" banner
  at startup. It is an opt-in declaration, not a detected fault.
- This GUI intentionally does **not** include any HCA gain-tuning controls —
  it's read-only monitoring.
- Only one ADC channel (ADC1, `PA0`, differential vs. `PA1`) is wired up in this
  firmware — no current/encoder channels like the 3-phase STM32F429 firmware
  this was ported from.

## Tests

```
go test ./...
```

Covers the frame round-trip (including checksum and sync rejection), the
trigger search, FFT calibration against a known tone, and ring-buffer wrap.

To also dump sample renders and a full-window screenshot for visual checks:

```
PLOT_OUT=/tmp/plots go test -run 'TestRenderPlotImage|TestWindowScreenshot' ./...
```
