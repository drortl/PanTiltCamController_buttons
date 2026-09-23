# Release Notes

All notable changes to this project are documented here, newest first.
Format: date, then each change with its status.

---

## 2026-09-23

### Fixed
- **10:49** — **Button thresholds re-centered using calibrated millivolts.**
  Raw `analogRead()` counts read low near 0V, so the S4 (pan-left) / S1
  (tilt-up) boundary had only ~1 count of margin. Buttons are now read with
  `analogReadMilliVolts()` (eFuse-calibrated), averaged over 64 samples.
  `BUTTON_ADC_THRESHOLDS` in [include/config.h](include/config.h) replaced by
  `BUTTON_MV_THRESHOLDS` = 48, 183, 384, 775, 2200 mV: the midpoints between
  meter readings of S4=0, S1=97mV, S2=268mV, S3=500mV, S5=1050mV, idle=~3.3V.
  The serial monitor (`a`) now prints `button_mv=` instead of `button_adc=`.
  Commit `20047d6`.
  **Status: flashed and confirmed working much better.**

---

## 2026-09-22

### Fixed
- **22:12** — **RS485 watchdog reset / random reboots.** `queryPositionDeg()`
  in [include/Pelco.h](include/Pelco.h) waited for the pan-tilt head's reply
  in a tight loop with no `yield()`. If the head was slow to answer (or not
  connected), the loop could starve the watchdog and reset the ESP32
  (`TG1WDT_SYS_RST`). Added a `yield()` call inside the wait loop.
  Commit `16528bc`.
  **Status: fix applied, build succeeds, awaiting flash + field confirmation.**

### Added (work in progress, currently stashed — not on `master`)
- **WiFi auto-off.** Turns WiFi off after an idle timer, configurable
  5-360 minutes (default 15) via a new switch in the web UI, plus a WiFi
  status icon (on/off) on the TFT.
  **Status: implemented and flashed once, but held back pending confirmation
  that it is unrelated to the crash-loop investigation above.**

### Fixed
- **22:35** — **Tilt-up button misread as pan-left.**
  `BUTTON_ADC_THRESHOLDS[0]` in [include/config.h](include/config.h) lowered
  16 → 8, and ADC sample averaging in `main.cpp` raised 16 → 64. Also fixed in
  hardware (ground wire routed too close to the S1 switch). This was applied
  and confirmed earlier in the session, lost during the crash-loop revert,
  and re-applied here. Commit `15db23a`.
  **Status: build succeeds, awaiting flash + field confirmation.**

### In progress (stashed, to be restored after the crash-loop is confirmed fixed)
- S5 (Home) button hold-duration retiming: Save Home now fires on release
  between 5-10 sec held; Clear Home fires at 20 sec held (previously
  4-15 sec and 25 sec).
- Power-outage persistence: pan/tilt position, home reference, and direction
  calibration now saved to flash (NVS) so they survive a reboot instead of
  resetting.

### Investigated, not adopted
- `WiFi.persistent(false)` in `setupWifi()` — tried as a candidate fix for
  the crash loop, made reset frequency worse. Reverted.

---

## Earlier history (from git log, before this file existed)

- `75665df` — Swap to ST7735 LCD + button pad, fix compass reliability
- `42facb8` — Home switch add func
- `31b69eb` — replace lcd to ftf 4" touch
- `cb74dc6` — update UI
- `4aed861` — Updates compass setting
- `1d84499` — Initial commit: ESP32-S3 pan-tilt camera controller
