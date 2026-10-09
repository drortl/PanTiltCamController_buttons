# Release Notes

What changed for the user, newest first.

**Downloads** (Android app and firmware):
https://github.com/drortl/PanTiltCamController_buttons/releases

---

## 2026-10-09 (2)

### Changed
- **WiFi settings moved to a Settings screen** in the web page and the app.
  Tap the ⚙ button to open it, and ← or the phone's Back button to return.
  The main screen now shows only the camera controls.
  **Status: flashed, installed and confirmed.**
- **Jog speed moved to the Settings screen** too, in the web page and the
  app (in the web page, with the pan and tilt step sizes).
  **Status: flashed, installed and confirmed.**
- **The app now matches the web page.** It has the 3 position presets (Go
  and Save) on the main screen, and the pan/tilt step sizes and limits on
  the Settings screen. The app and the web page always show the same
  values - a change made in one appears in the other.
  **Status: flashed, installed and confirmed.**

### New
- **Android app for off-grid control.** Control the camera from your phone
  over Bluetooth - no WiFi or internet needed. It connects by itself when
  opened and has the same controls as the web page. Hold a direction button
  to keep moving.
  **Status: installed and confirmed.**
- **WiFi settings in the web page and the app.** You can change the home
  WiFi name and password, and the access point name and password. Leave a
  password empty to keep the current one. The device restarts to apply.
  **Status: flashed and confirmed.**
- **Turn WiFi on or off from the app.** With WiFi off, the camera is
  controlled by Bluetooth only. Bluetooth always stays on, so you can turn
  WiFi back on from the app.
  **Status: flashed and confirmed.**
- **Switch between access point and home WiFi.** A new switch in the web
  page and the app. If the home WiFi is not found, the device starts its
  own access point again, so you can always reach it.
  **Status: flashed and confirmed.**
- **Bluetooth control from a web page.** Open
  https://drortl.github.io/PanTiltCamController_buttons/ble/ in Chrome
  (Android or PC, not iPhone) and connect to "PanTiltCam".
  **Status: flashed and confirmed.**

---

## 2026-10-09

### Changed
- **Go to Azimut, Pan Mid and Tilt Zero move smoothly.** The camera now
  turns in one smooth motion instead of many small steps.
  **Status: flashed and confirmed.**
- **WiFi works as an access point.** Connect to WiFi "PanTiltCam-Setup"
  (password 12345678) and open http://192.168.4.1. Startup is also faster.
  **Status: flashed and confirmed.**
- **Clear Home is now 5 short presses on the Home button.** Before, you had
  to hold the button for 25 seconds. The screen counts the presses
  ("CLEAR 2/5" ...). One press is still Home, and a 4-15 second hold is
  still Save Home.
  **Status: flashed and confirmed.**
- **Position is kept after a power cut.** After power comes back, the
  camera position and azimuth are correct, without pressing Home first.
  **Status: flashed and confirmed.**

### Removed
- **Auto Calibrate.** It is not needed any more: the pan and tilt limits
  are now fixed (pan 0-350°, tilt 0-50°). The web page still has Pan Mid
  and Tilt Zero.
  **Status: flashed and confirmed.**

### Fixed
- **Go to Azimut got stuck or turned back and forth** for some targets
  (for example 350 or 200). It now goes straight to the target, and takes
  the long way round when the short way is blocked by the pan limit.
  **Status: flashed and confirmed.**
- **Go to Azimut did not move the camera** even though the page said "ok".
  The web page now also shows a message when Go to Azimut can't run.
  **Status: flashed and confirmed.**

---

## 2026-09-24

### Fixed
- **The screen blinked about twice a second.** It now updates without
  blinking.
  **Status: flashed and confirmed.**

### Changed
- **Brighter screen colors for outdoor use.** Azimut and error text are now
  white.
  **Status: flashed and confirmed.**

---

## 2026-09-23

### Fixed
- **Buttons were sometimes misread** (for example, tilt-up acted as
  pan-left). Button reading is now much more reliable.
  **Status: flashed and confirmed.**

---

## 2026-09-22

### Fixed
- **Random restarts** when the pan-tilt head was slow to answer or not
  connected.
  **Status: waiting for field confirmation.**
- **Tilt-up button acted as pan-left.** Also fixed a ground wire that ran
  too close to the button.
  **Status: waiting for field confirmation.**

---

## Earlier

- Switched to the small ST7735 screen and the 5-button pad, and made the
  compass more reliable.
- Added the Home button functions.
- First version: ESP32-S3 pan-tilt camera controller with web page and
  compass.
