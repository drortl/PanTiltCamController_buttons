#pragma once

// ============================================================================
// WIRING / PIN TABLE — ESP32-S3-WROOM-1 N16R8 (44-pin devkit, Octal PSRAM)
// ============================================================================
// GPIO 0, 3, 45, 46        strapping pins - avoid
// GPIO 19, 20              native USB D-/D+ - avoid
// GPIO 22-25               do not exist on the ESP32-S3 chip (silicon gap)
// GPIO 26-32               SPI flash (all ESP32-S3 modules) - reserved
// GPIO 33-37               Octal PSRAM (N16R8 specifically) - reserved
// GPIO 39-44               JTAG / UART0 (USB-serial "COM" port) - avoid
// GPIO 48                  onboard WS2812 RGB LED
//
// Signal                   GPIO   Notes
// ----------------------------------------------------------------------------
// TFT CS                   5      ST7796S, 480x320, 4-wire SPI (module: MSP4021)
// TFT DC/RS                17
// TFT RST                  16
// TFT SCK                  12     shared SPI bus (display + touch), explicit (not core default)
// TFT SDI/MOSI             11     shared SPI bus (display + touch)
// TFT SDO/MISO             18     shared SPI bus - only actually driven by touch reads
// TFT LED (backlight)      6      driven HIGH = on
// Touch T_CS               7      XPT2046, same SPI bus as TFT, own chip select
// Touch T_IRQ              8      low when touch panel is pressed
// Compass SDA               21    QMC5883P, I2C addr 0x2C
// Compass SCL               14
// RS485 RX  (module RO)      4    UART2, Pelco-D - to the pan-tilt unit
// RS485 TX  (module DI)      13   UART2, Pelco-D - to the pan-tilt unit
// RS485 DE/RE (module DE+/RE) 15  tie module's DE and /RE pins together here
// ============================================================================
// The pan/tilt motion itself is handled entirely by an external
// self-contained pan-tilt head (internal stepper + MCU, PELCO-D/P
// auto-adapting, address set via onboard DIP switches 1-6, baud via
// switches 7-8) - the ESP32 only talks to it over RS485, it does not
// drive any local motors.
// ============================================================================

// ---------- WiFi (joins an existing network) ----------
// Tries the primary network first, then the backup, at boot. If neither is
// reachable, falls back to hosting its own access point (AP_SSID/AP_PASSWORD)
// so the device is never completely unreachable.
// Actual credentials live in secrets.h (gitignored, not committed) - copy
// secrets.h.example to secrets.h and fill in your real network details.
#include "secrets.h"
#define WIFI_CONNECT_TIMEOUT_MS 10000  // per network, before trying the next

#define AP_SSID         "PanTiltCam-Setup"
#define AP_PASSWORD     "12345678"   // min 8 chars, required by WiFi.softAP

// ---------- TFT (ST7796S, 480x320, 4-wire SPI) + XPT2046 touch ----------
// Display and touch chip share one physical SPI bus (SCK/MOSI/MISO) with
// separate chip-selects - see TouchUI.h. MISO is only actually driven by
// the touch chip (the display is write-only), but is wired through anyway
// since it's the same 4 header pins on the module either way.
// Explicit pins throughout (not the ESP32-S3 hw-SPI defaults) because the
// default MISO (GPIO13) is already RS485_TX_PIN on this board.
#define TFT_CS_PIN      5
#define TFT_DC_PIN      17
#define TFT_RST_PIN     16
#define TFT_SCK_PIN     12
#define TFT_MOSI_PIN    11
#define TFT_MISO_PIN    18
#define TFT_BLK_PIN     6
#define TOUCH_CS_PIN    7
#define TOUCH_IRQ_PIN   8

// Raw XPT2046 ADC range (0-4095) mapped to screen pixels - resistive touch
// panels vary unit to unit, these are typical defaults for this module.
// If touch feels off (dead zones at the edges, or reversed axes), watch the
// "touch raw=.." line TouchUI.h prints to Serial on each press and adjust
// these to match what you actually see at the screen corners; swap the min/
// max of an axis if it reads backwards.
#define TOUCH_RAW_MINX  300
#define TOUCH_RAW_MAXX  3800
#define TOUCH_RAW_MINY  300
#define TOUCH_RAW_MAXY  3800
// The touch chip's raw X/Y are wired to the resistive film's physical axes,
// not to the display's setRotation() - if touches consistently land with X
// and Y transposed (e.g. dragging left-right on screen moves the touch
// point up-down), set this to 1 instead of adjusting MINX/MAXX/MINY/MAXY.
#define TOUCH_SWAP_XY   0

// This project's original 1.8" TFT needed 4MHz for signal integrity on
// breadboard jumper wires - starting this new, much larger panel at the
// same proven-safe speed rather than guessing higher. Note this only
// controls draw calls made AFTER tft.init() (see TouchUI::begin()) - the
// display's one-time init command sequence always runs at the Adafruit
// ST7735/ST7789 library's own hardcoded 32MHz, which this can't override;
// if the screen stays blank/white, that fixed-speed init sequence failing
// on the wiring is the most likely cause. Raise this once wiring is
// confirmed solid (a full-screen redraw at 4MHz is ~0.6s, visibly slow for
// a touch UI).
#define TFT_SPI_SPEED_HZ 4000000

// ---------- Digital compass (QMC5883P, I2C) ----------
#define COMPASS_SDA_PIN 21
#define COMPASS_SCL_PIN 14
#define QMC5883P_ADDR   0x2C

// ---------- RS485 (Pelco-D, via UART2) ----------
// TTL-to-RS485 module: WeAct ISORS485 (CA-IS2092A, isolated). Its RS485-bus
// side (A/+, B/-) is galvanically isolated from its logic side, which has
// its own pinout:
//   5V   -> 5V              (powers the isolated side, needs 4.5-5.5V)
//   VIO  -> 3V3              (logic-level reference for DI/DE/RE/RO)
//   G    -> GND
//   DI   -> RS485_TX_PIN      (data the ESP32 transmits onto the bus)
//   RO   -> RS485_RX_PIN      (data the ESP32 receives from the bus)
//   DE   -> RS485_DE_RE_PIN  \_ tied together - HIGH = transmit, LOW = receive
//   /RE  -> RS485_DE_RE_PIN  /  (this module exposes them as separate pins;
//                                driving both from one GPIO is the standard way
//                                to use them, and is what PelcoController expects)
// Talks to an external pan-tilt head (own internal motor + MCU) whose
// address is set via its onboard DIP switches 1-6, baud via switches 7-8.
#define RS485_RX_PIN     4
#define RS485_TX_PIN     13
#define RS485_DE_RE_PIN  15
#define RS485_BAUD       2400   // common Pelco default - change to match the pan-tilt unit's DIP setting
#define RS485_ADDRESS    1      // pan-tilt unit's Pelco address (its DIP switches 1-6)

#define PELCO_HOME_PRESET 1     // preset number the pan-tilt head calls on "HOME"

// Pelco-D has no "move by N degrees" command - only "move until stopped".
// The head does answer a non-standard position-query extension (see
// queryPositionDeg() in Pelco.h), used to correct drift after each move,
// but there's still no "move to N degrees" command - each button press is
// turned into a fixed PELCO_STEP_DEG move by sending a move command at the
// current jog speed (adjustable via the web UI, see jogSpeed in main.cpp)
// for a computed duration, then stopping: duration scales with jog speed
// since the head's angular rate is assumed proportional to the speed byte.
// PELCO_MAX_PAN/TILT_SPEED_DEG_S are the unit's rated max speed (14 deg/s
// pan, 2 deg/s tilt at speed byte 63, per its manual) - re-measure against
// a known angle and adjust if the actual step size drifts from 5 degrees.
#define PELCO_STEP_DEG             5.0f
#define PELCO_STEP_SPEED           63     // default jog speed (0-63), adjustable via web UI
#define PELCO_MAX_PAN_SPEED_DEG_S  14.0f
#define PELCO_MAX_TILT_SPEED_DEG_S 2.0f

// Closed-loop drives (Go to Azimut, Pan Mid, Tilt Zero - see updateGoTo()):
// full PELCO_STEP_DEG pulses at jogSpeed would only ever land within half a
// step (2.5 deg) of the target, since there's no smaller final-correction
// pulse - so once within AUTO_SLOWDOWN_THRESHOLD_DEG of the target, pulses
// shrink to AUTO_CREEP_STEP_DEG (clamped to whatever distance remains) at
// the slower AUTO_CREEP_SPEED, letting the final approach land within
// AUTO_ARRIVE_TOLERANCE_DEG instead. 0.1 deg is close to the floor of what's
// achievable - at that point pulse durations are tens of ms and the head's
// own motor start/stop lag (not commanded distance) dominates; MIN_PULSE_MS
// in stepMsFor() keeps those pulses from rounding down to ~0 and doing
// nothing, but if it doesn't converge in practice, raise this back up.
#define AUTO_ARRIVE_TOLERANCE_DEG   0.1f
#define AUTO_SLOWDOWN_THRESHOLD_DEG 3.0f
#define AUTO_CREEP_STEP_DEG         1.0f
#define AUTO_CREEP_SPEED            10     // slow speed byte (0-63) for the final approach

// ---------- Auto Calibration (pan/tilt travel limits) ----------
// Drives each axis to its mechanical end and detects arrival by the head's
// real reported position going flat/stalled, then repeats for the opposite
// end and the other axis - same approach as the sibling PanTiltRS485Controller
// USB app's Auto Calibrate (see MainForm.cs there). Limits are stored as raw
// head-reported degrees and persisted to flash.
#define CAL_SPEED               63     // full speed for the calibration scan
#define CAL_STALL_WINDOW_MS     900    // must be flat for this long to count as arrived
#define CAL_STALL_EPSILON_DEG   0.6f
#define CAL_PHASE_TIMEOUT_MS    100000 // safety backstop per phase (tilt is slow)

// Fixed calibration-target buttons in the web UI: drive straight to a known
// raw motor position (same frame queryPositionDeg() reads in) rather than
// scanning to a mechanical limit. Measured on this unit - pan center is
// 175 deg, tilt level/horizontal (zero) is 29 deg - re-measure and adjust
// if re-mounted.
#define PAN_MID_TARGET_DEG      175.0f
#define TILT_ZERO_TARGET_DEG    29.0f
#define GOTO_DRIVE_TIMEOUT_MS   60000  // safety backstop (tilt is slow over a large range)

// ---------- User position presets ----------
// Preset 1 (PELCO_HOME_PRESET) is reserved for Home. Presets 2-4 are
// exposed in the web UI as three user-assignable "save current position /
// go to it" slots (uses the head's own preset memory via setPreset/callPreset).
#define PELCO_USER_PRESET_COUNT 3
#define PELCO_USER_PRESET_BASE  2

// ---------- Shared state enums ----------
// Defined here (rather than in main.cpp) so both main.cpp and TouchUI.h can
// declare `extern` globals of these types without one having to include the
// other.
enum CalPhase { CAL_IDLE, CAL_PAN_MIN, CAL_PAN_MAX, CAL_TILT_MIN, CAL_TILT_MAX };
enum GoAxis { GO_NONE, GO_PAN, GO_TILT };
enum GoSource { GO_SRC_NONE, GO_SRC_AZIMUTH, GO_SRC_PAN_MID, GO_SRC_TILT_ZERO };
// Result codes for doGoAzimuth(), since it has two distinct failure modes
// the caller needs to tell apart (the web UI maps them to different HTTP
// statuses; the touch UI just ignores the distinction and no-ops).
enum GoAzimuthResult { GOAZ_OK, GOAZ_NO_HOME, GOAZ_NO_REPLY };
