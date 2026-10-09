# Release Notes

All notable changes to this project are documented here, newest first.
Format: date, then each change with its status.

---

## 2026-10-09 (test)

### Added
- **Bluetooth LE control (test).** The ESP32-S3 has BLE only, so the WiFi
  web page can't run over Bluetooth. Instead the device now advertises a BLE
  service as `PanTiltCam` (`setupBle()` in `main.cpp`, runs alongside WiFi),
  and a new Web Bluetooth page `ble/index.html` sends text commands to it
  (step, stop, home, save/clear home, go to azimut, pan mid, tilt zero,
  speed) and reads a compact status. Works in Chrome/Edge on Android and
  PC, not on iPhone. Range is set by `BLE_TX_POWER` in `config.h` (0 dBm).
  Jog and Go to Azimut logic is shared with the web UI (`doStep()`,
  `doGoAzimuth()`). BLE adds ~21 KB RAM and ~590 KB flash.
  **Status: built, not flashed.**

---

## 2026-10-09

### Changed
- **14:23** — **Go to Azimut / Pan Mid / Tilt Zero now move in one smooth
  motion.** Before, the drive was a series of pulses (move, stop, read
  position, repeat), so the head moved in visible steps. Now `updateGoTo()`
  starts a "cruise": one continuous move at jog speed, reading the position
  every 100 ms while moving (`updateCruise()`). It stops at a braking
  distance before the target (distance covered in `AUTO_BRAKE_LEAD_S` =
  0.4 sec at the current speed, min 3°), waits 300 ms for the head to
  settle, then slow creep pulses finish the last few degrees. The cruise
  also stops at a limit or after passing the target, flips the learned
  direction if the head moves away from the target, and gives up if the head
  does not move for 1.5 sec (e.g. blocked). New settings in `config.h`:
  `AUTO_CRUISE_POLL_MS`, `AUTO_BRAKE_LEAD_S`, `AUTO_SETTLE_MS`,
  `AUTO_STALL_MS`. Commit `7a77029`.
  **Status: flashed and confirmed.**
- **14:23** — **WiFi is now access-point only.** The device always hosts
  its own AP `PanTiltCam-Setup`, password `12345678`, web UI at
  `http://192.168.4.1`. It no longer tries to join the home networks in
  `secrets.h` (this also removes the up to 20 sec boot delay when they are
  not in range). Set `WIFI_AP_ONLY` to `0` in `config.h` to go back to the
  old behavior. The TFT shows the AP name instead of "AP mode".
  Commit `7a77029`.
  **Status: flashed and confirmed.**
- **14:23** — **Clear Home on the Home button (S5) is now 5 short presses,
  not a 25 sec hold.** The 1st press runs Home as before; presses 2-4 only
  count (TFT shows "CLEAR 2/5" ... "CLEAR 4/5"); the 5th press runs Clear
  Home. If the next press starts more than 1 sec after the last release,
  the count restarts. Save Home (hold 4-15 sec) is unchanged. Settings:
  `CLEAR_HOME_PRESS_COUNT`, `HOME_MULTI_PRESS_GAP_MS` in `config.h`. Also
  removed the old unused `HOME_HOLD_*` defines and the duplicate
  `STATUS_MESSAGE_MS`. Commit `7a77029`.
  **Status: flashed and confirmed.**
- **14:23** — **Position and home reference now survive a power loss.**
  Before, pan/tilt position reset to 0/0 (unknown) at boot, but the home
  azimuth was restored, so Azimut showed the home heading even when the
  camera pointed elsewhere. Now `main.cpp` also saves to flash (NVS):
  - the head's raw position at Save Home (`panRaw0`, `tiltRaw0`), so drift
    correction works after a reboot;
  - the current pan/tilt position (`panPos`, `tiltPos`, `posOk`), written
    2 sec after movement stops (not on every step, to limit flash wear).

  At boot the saved position is restored. When the head first answers a
  position query, the position is rebuilt from the head's real raw reading.
  Clear Home removes the saved raw-at-home reference. Commit `7a77029`.
  **Status: flashed and confirmed.**

### Removed
- **14:23** — **Auto Calibrate, Cancel and Set Min/Max (web UI) - limits
  are now fixed.** The head's range never changes, so the limits are set in
  `config.h`: pan 0-350°, tilt 0-50° (`PAN_LIMIT_*`, `TILT_LIMIT_*`).
  Removed the calibration state machine, the `/autoCalibrate`,
  `/cancelCalibrate` and `/calLimit` endpoints, and loading/saving limits in
  flash (old `panMin`/`panMax`/`tiltMin`/`tiltMax` keys are ignored). The
  web panel is now "POSITION TARGETS" with Pan Mid / Tilt Zero and the fixed
  limits shown read-only. Go to Azimut always keeps pan inside 0-350° and
  takes the long way round when the short way would cross the 350-360°
  dead zone. Commit `7a77029`.
  **Status: flashed and confirmed.**

### Fixed
- **14:23** — **Go to Azimut (e.g. 0 → 350 or 200) got stuck and turned
  back the opposite way.** `updateGoTo()` flipped its learned motor
  direction whenever the gap to the target grew - including after an
  overshoot past the target, which is the right direction. The flipped sign
  then drove away, flipped back, and so on. Now it flips only when the head
  moved away on the same side of the target. Serial log prints each
  direction flip. Commit `7a77029`.
  **Status: flashed and confirmed.**
- **14:23** — **Go to Azimut answered "ok" but the motor did not move.**
  The pan limit check used the dead-reckoned raw estimate
  `panRawAtHome + panPositionDeg` without wrapping to [0, 360), so e.g.
  350 + 20 = 370 read as past the max limit and the drive stopped before
  the first pulse. The raw go-to target (`currentRaw + diff`) was not
  wrapped either. Now the estimate is wrapped, the go-to drive checks
  limits against the fresh raw reading (`panAtLimitRaw()` /
  `tiltAtLimitRaw()`), and the target is wrapped and clamped into the pan
  range. Serial log shows each go-to decision. The web page now shows an
  alert when the request fails or the target was limited by the pan range.
  Commit `7a77029`.
  **Status: flashed and confirmed.**

---

## 2026-09-24

### Fixed
- **15:11** — **TFT blinked about every 0.5 sec.** `updateDisplay()` in
  `main.cpp` cleared the whole lower area with `fillRect` before each redraw.
  At 4 MHz SPI this took ~60 ms and showed as a black flash, and compass noise
  (≥0.5° change) triggered a redraw on most refresh cycles. Text is now drawn
  with a black background and padded to a fixed width, so each line overwrites
  its old pixels in place. The status line is cleared only when its text
  changes. Commit `acc3c52`.
  **Status: flashed and confirmed - much better.**

### Changed
- **15:11** — **Brighter TFT colors for outdoor use.** Azimut text cyan →
  white. Error text ("NO COMPASS", "Pan ??", "Tilt ??") red → white (orange
  was tried and rejected). Pan/Tilt stay yellow, status stays green.
  Commit `acc3c52`.
  **Status: flashed and confirmed.**

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
