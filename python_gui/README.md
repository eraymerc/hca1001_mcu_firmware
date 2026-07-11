# HCA1001 ADC Monitor (Python GUI)

Dark-mode desktop app that connects to the STM32F429I-DISC1 over its **USB
CDC virtual COM port** (no UART wiring — see
`USB_DEVICE/App/usbd_cdc_if.c`), and plots Voltage / Current / Encoder /
HCA control-loop error live.

## Install

```
pip install -r requirements.txt
```

## Run

```
python main.py
```

## Usage

1. Plug the board in over USB, flash the updated firmware.
2. Click **Refresh** if your port isn't listed, select it in the **COM
   Port** dropdown, click **Connect**.
3. Click **Test Connection** to confirm you picked the right port — the
   firmware replies with an identification string; the status bar will
   say `Device identified: HCA1001_ADC_STREAM_V1`. If nothing happens
   within a second, you've likely got the wrong port or the firmware
   isn't flashed yet.
4. Click **Start Streaming**. Each row (Voltage, Current, Encoder, HCA
   Error) plots live at ~1 kHz sample rate (decimated on-device from the
   40 kHz control loop — see `ADC_STREAM_DECIMATION` in `main.c`).
5. Click **FFT** next to any signal to open a live spectrum window for
   just that signal (2048-sample window, Hann-windowed, updates 4×/sec,
   Nyquist = 500 Hz — enough to see harmonics up to the 9th of a 50 Hz
   fundamental).
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
  controls — it's read-only monitoring, per the current scope.
- Wire protocol lives in `protocol.py` and must match
  `AdcStreamFrame_t` in `Core/Inc/main.h` byte-for-byte. If you change
  the frame layout on the firmware side, update `FRAME_FORMAT` here.
