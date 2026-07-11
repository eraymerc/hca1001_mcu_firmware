# USB ADC Streaming — CubeMX Guidance

This covers what I changed in firmware for the ADC/HCA-error streaming
feature, and — per your instruction — what you should check/change in
CubeMX yourself rather than me hand-editing generated middleware.

## What I changed (application code only)

All changes are inside `USER CODE BEGIN/END` blocks, so regenerating from
the `.ioc` file is safe and won't remove them:

- `Core/Inc/main.h` — `AdcStreamFrame_t` frame layout, command bytes,
  `extern volatile uint8_t streaming_enabled`.
- `Core/Src/main.c` — ring buffer, frame packing/checksum, decimation
  (40 kHz control loop → 1 kHz USB stream), and draining the ring buffer
  to `CDC_Transmit_HS()` in the main `while(1)` loop.
- `USB_DEVICE/App/usbd_cdc_if.c` — `CDC_Receive_HS()` now parses the
  single-byte commands `'S'`/`'X'`/`'P'` (start/stop/ping).

I did **not** touch `Middlewares/ST/STM32_USB_Device_Library/*`,
`USB_DEVICE/Target/usbd_conf.c`, or `USB_DEVICE/App/usbd_desc.c` — those
are CubeMX-generated and should stay generated.

## 1. IRQ priority — please fix this, it affects control-loop jitter

I checked the `.ioc`:

```
NVIC.OTG_HS_IRQn        = enabled, preempt priority 0, sub-priority 0
NVIC.DMA2_Stream0/1/2   = enabled, preempt priority 2, sub-priority 0
```

Lower number = higher priority on Cortex-M. Right now **USB has higher
priority than the ADC/DMA interrupts that drive your 40 kHz HCA control
loop**. That means every USB interrupt (start-of-frame every 1 ms, plus
IN/OUT completions — now more frequent since streaming adds regular
traffic) can preempt `HAL_ADC_ConvCpltCallback()` and delay
`HCA_Process()` / `USPWM()`, adding jitter to the control loop.

This was already a latent risk before my changes, but streaming makes USB
IRQs fire much more regularly, so it's worth fixing now:

**In CubeMX:**
1. Open `hca1001-stm32f429zit6-disc1-1faz.ioc`.
2. Go to the **NVIC** tab (Pinout & Configuration → System Core → NVIC,
   or under USB_OTG_HS's own config, "NVIC Settings" sub-tab).
3. Find **USB On The Go HS global interrupt**.
4. Change its **Preemption Priority** from `0` to something numerically
   *higher* than `2` (e.g. `5`), so it can never preempt
   `DMA2_Stream0/1/2` (priority `2`).
5. Regenerate code (Project → Generate Code). This only touches the
   generated NVIC init call in `usbd_conf.c`, which is outside the
   USER CODE regions I edited, so nothing of mine is affected.

If you'd rather not touch NVIC config, an equivalent alternative is
raising the DMA stream priorities in CubeMX (e.g. to `0`) instead of
lowering USB — either way, the control ISR must not be preemptable by USB.

## 2. Things that are already correctly configured (no action needed)

- **USB peripheral**: `USB_OTG_HS` in `Device_Only_FS` virtual mode
  (embedded full-speed PHY, no external ULPI) on `PB14`/`PB15`. This is
  USB, not UART — matches your requirement.
- **USB Device middleware**: `CDC_HS` class enabled.
- **48 MHz USB clock**: confirmed present (`PLLQ = 7` → 48 MHz), required
  for the OTG_HS peripheral to run in FS-PHY mode.
- **VBUS sensing**: disabled (`vbus_sensing_enable = DISABLE`) — correct
  if your board doesn't wire a VBUS-detect pin to the MCU; the device
  just assumes it's powered whenever the MCU is running.

## 3. Optional CubeMX check if you see dropped/garbled frames

`APP_RX_DATA_SIZE` / `APP_TX_DATA_SIZE` in `usbd_cdc_if.h` are 2048 bytes
— comfortably larger than a single 27-byte frame, so no change needed
there. If you ever increase the frame size a lot, that's the CubeMX-owned
constant to watch (Middleware → USB_DEVICE → Class Parameters).

## 4. Rebuild/flash

After any CubeMX regeneration:
```
make clean && make -j
```
then flash as usual (e.g. `st-flash`, STM32CubeProgrammer, or your IDE).

## 5. Sensor scaling you'll want to fill in later

`main.c` already has a calibrated conversion for the voltage channel
(`adcToVoltsActual()`, using `VDIV_RATIO`/`OPAMP_GAIN`/`V_PEAK_NOM`). I
did **not** invent scaling for the current and encoder channels since I
don't know your sensor gains — they're streamed as raw ADC pin voltage
(0–3.3 V). Once you know those sensors' V/A and V/rad (or similar)
factors, adjust the `current_v`/`encoder_v` calculation in
`HAL_ADC_ConvCpltCallback()` (`Core/Src/main.c`) — that's plain
application code, not middleware, so feel free to edit it directly.
