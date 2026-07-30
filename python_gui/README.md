# HCA1001 ADC Monitor (Python GUI)

Dark-mode desktop app that connects to the STM32G474RE over the **ST-LINK's
virtual COM port** (LPUART1, 2,097,000 baud — see `Core/Src/main.c`) and
plots the single ADC1 Voltage reading plus the HCA control-loop error live.
Just the one USB cable into the Nucleo's ST-LINK connector — no external
UART adapter needed.

## Wire protocol

One 19-byte frame per sample (sync + seq + timestamp + float voltage +
float error + checksum), decimated to 1kHz from the 40kHz control loop
(`ADC_STREAM_DECIMATION` in `main.c`). Full float precision, resyncs
instantly on any dropped/corrupted byte via the two sync bytes.

## Install

```
pip install -r requirements.txt
```

## Run

```
python main.py
```

## Usage

1. Flash the firmware, plug the Nucleo into USB (ST-LINK cable).
2. Click **Refresh** if your port isn't listed, select it in the **COM
   Port** dropdown, click **Connect**.
3. Click **Test Connection** to confirm you picked the right port — the
   firmware replies with an identification string; the status bar will
   say `Device identified: HCA1001_ADC_STREAM_V1`. If nothing happens
   within a second, you've likely got the wrong port or the firmware
   isn't flashed yet.
4. Click **Start Streaming**. Each row (Voltage, HCA Error) plots live at
   ~1 kHz.
5. Click **FFT** next to any signal to open a live spectrum window for
   just that signal (2048-sample window, Hann-windowed, updates 4×/sec,
   Nyquist = 500 Hz).
6. Use the checkboxes on each row to choose which signals are included
   when exporting.
7. **Save Selected as CSV** — writes every captured sample (seq,
   timestamp_ms, and the checked signal columns) since you connected.
8. **Save Selected as PNG** — exports one PNG per checked signal's plot
   into a folder you choose.

## Notes

- The session buffer for CSV export is capped at 2,000,000 samples
  (~33 minutes at 1 kHz) to bound RAM use. Save and reconnect to keep
  capturing past that.
- This GUI intentionally does **not** include any HCA gain-tuning
  controls — it's read-only monitoring.
- Wire protocol lives in `protocol.py` and must match `AdcStreamFrame_t`
  in `Core/Inc/main.h` byte-for-byte. If you change the frame layout on
  the firmware side, update `FRAME_FORMAT` here.
- Only one ADC channel (ADC1, `PA0`, differential vs. `PA1`) is wired up
  in this port — no current/encoder channels like the 3-phase STM32F429
  firmware this was ported from.
- LPUART1 is shared with the ST-LINK's VCP hardware, which is why no
  separate external UART adapter or extra wiring is needed — CubeMX's
  `BSP_COM_Init`-based VCP setup was replaced with a direct LPUART1
  peripheral config so the baud rate could be raised past 115200.
