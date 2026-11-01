# HCA1001 ADC Monitor (Python GUI)

Dark-mode desktop app that connects to the STM32G474RE over the **ST-LINK's
virtual COM port** (LPUART1, 8,000,000 baud — see `Core/Src/main.c`) and
plots the single ADC1 Voltage reading plus the HCA control-loop reference
and error live, at the full 40kHz loop rate.
Just the one USB cable into the Nucleo's ST-LINK connector — no external
UART adapter needed.

## Wire protocol

One 59-byte frame per 8 samples (sync + seq + timestamp + 8x[int16
voltage, int16 reference, int16 error] + checksum), one sample per 40kHz
control-loop tick -- 295,000 B/s, about 37% of the link. Resyncs
instantly on any dropped/corrupted byte via the two sync bytes.

Samples are int16 fixed point, finer than one ADC code: voltage in
1/16 V, reference and error in 1/2048 p.u. (`STREAM_VOLT_SCALE` and
`STREAM_PU_SCALE` in `main.c` and `protocol.py`).

`seq` advances for every batch the device produces, including one it had
to drop, so any missing samples show up as a gap in `seq`. When cutting a
capture out of a recording, use a stretch with contiguous `seq`.

Coefficient frames (HCA Kp/Ki) share the link, distinguished by a
different second sync byte (0x5B vs 0x5A).

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
4. Click **Start Streaming**. Each row (Voltage, HCA Reference, HCA
   Error) plots live at 40 kHz. The readout beside the connection status shows the measured
   stream rate and any frame loss; if it turns red the spectrum's
   frequency axis is scaled wrong.
5. Click **FFT** next to any signal to open a live spectrum window for
   just that signal (16384-sample window, Hann-windowed, updates 4×/sec,
   Nyquist = 20 kHz, harmonics tabulated and THD summed to H50).
6. Use the checkboxes on each row to choose which signals are included
   when exporting.
7. **Save Selected as CSV** — writes every captured sample (seq,
   timestamp_ms, and the checked signal columns) since you connected.
8. **Save Selected as PNG** — exports one PNG per checked signal's plot
   into a folder you choose.
9. **Record** — captures the checked signals for the time set beside it
   (default 0.1 s = five 50 Hz periods, up to 50 s), then writes
   `rec_<time>.csv` (seq, timestamp_ms, t_s and the signal columns) plus
   one `rec_<time>_<signal>.png` per signal into a folder you choose.
   Streaming must be running. A gap in `seq` throws the partial capture
   away and starts again (up to 5 times), so a saved recording never has
   missing samples. **Start at ref. zero crossing** waits for the
   reference's rising zero crossing first, so every capture starts at 0°.

## Notes

- The session buffer for CSV export is capped at 2,000,000 samples
  (50 seconds at 40 kHz) to bound RAM use. Save and reconnect to keep
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
