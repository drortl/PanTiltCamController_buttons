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
// RS485 RX  (module RO)    4      UART2, Pelco-D - to the pan-tilt unit
// RS485 TX  (module DI)    5      UART2, Pelco-D - to the pan-tilt unit
// RS485 DE/RE (DE+/RE)     6      tie module's DE and /RE pins together here
// Compass SDA              7      QMC5883P, I2C addr 0x2C
// Compass SCL              15
// TFT BLK                  16     backlight, driven HIGH = on
// TFT CS                   17     ST7735, 128x160
// TFT DC                   18
// TFT RST                  8
// Button ADC                9     five-button resistor-ladder pad
// TFT SDA  (hw SPI, fixed) 11     module's SPI data pin; ESP32-S3 default SPI pin
// TFT SCL  (hw SPI, fixed) 12     module's SPI clock pin; ESP32-S3 default SPI pin
//
// All 12 signals above live on header J1 alone, grouped into contiguous
// runs by peripheral (RS485 -> Compass -> TFT) for easy hand-soldering -
// nothing is wired to the opposite header (J3) anymore. GPIO3/GPIO46, which
// fall physically in the middle of that run, are strapping pins and stay
// unconnected (see the reserved list above) - that gap is unavoidable on
// any layout. TFT's hw-SPI SCL/SDA keep the ESP32-S3's default pins, a
// short reach past TFT RST on the same row.
// ============================================================================
// The pan/tilt motion itself is handled entirely by an external
// self-contained pan-tilt head (internal stepper + MCU, PELCO-D/P
// auto-adapting, address set via onboard DIP switches 1-6, baud via
// switches 7-8) - the ESP32 only talks to it over RS485, it does not
// drive any local motors.
// ============================================================================

// ---------- WiFi ----------
// WIFI_AP_ONLY 1: the device always hosts its own access point
// (AP_SSID/AP_PASSWORD), web UI at http://192.168.4.1 - no router needed.
// WIFI_AP_ONLY 0: tries the primary network first, then the backup, at
// boot. If neither is reachable, falls back to the access point above so
// the device is never completely unreachable.
#define WIFI_AP_ONLY 1
// Actual credentials live in secrets.h (gitignored, not committed) - copy
// secrets.h.example to secrets.h and fill in your real network details.
#include "secrets.h"
#define WIFI_CONNECT_TIMEOUT_MS 10000  // per network, before trying the next

#define AP_SSID         "PanTiltCam-Setup"
#define AP_PASSWORD     "12345678"   // min 8 chars, required by WiFi.softAP

// ---------- TFT (ST7735, 128x160, hardware SPI) ----------
#define TFT_CS_PIN      17
#define TFT_DC_PIN      18
#define TFT_RST_PIN     8
#define TFT_BLK_PIN     16
// SCL=12, SDA=11 (the module's labels for SPI clock/data - not I2C) are the
// ESP32-S3 default hardware SPI pins, used automatically by the library
// since no explicit SPI pins are passed.
// If colors look wrong/inverted, switch INITR_BLACKTAB to INITR_GREENTAB in main.cpp.

// ---------- Five-button analog input module ----------
// The module uses one resistor ladder output. Nominal resistance from OUT to
// GND is S4=0, S1=330 ohm, S2=940 ohm, S3=1.9 kohm, S5=5 kohm, but live ADC
// capture (2026-09-22) showed the S4/S1 boundary sits much lower than the
// nominal-resistance math predicts - S4 reads 0-7 and S1 reads 8-26 (ESP32
// ADC is noisy/non-linear this close to 0V), not the ~0 vs ~130 the resistor
// values alone would suggest. The old threshold of 16 sat inside S1's own
// noise band, so it intermittently misread tilt-up (S1) as pan-left (S4).
// Thresholds are now in calibrated millivolts (analogReadMilliVolts), which
// fixes that low-end error. Meter readings (2026-09-23) with the module's
// ~11 kohm pull-up to 3.3V: S4=0, S1=97mV, S2=268mV, S3=500mV, S5=1050mV,
// idle=~3.3V. Each threshold is the midpoint between neighbouring buttons.
// GPIO9 (not GPIO1) so it stays in the ADC1 range (GPIO1-10) - ADC2 isn't
// reliable while WiFi is active.
#define BUTTON_ADC_PIN 9
#define BUTTON_THRESHOLD_COUNT 5
static const uint16_t BUTTON_MV_THRESHOLDS[BUTTON_THRESHOLD_COUNT] = {
	48, 183, 384, 775, 2200
};

// ADC classes are ordered by resistance, not by the labels printed on the
// module: 0 ohm is S4, followed by S1, S2, S3, and S5.
static const uint8_t BUTTON_CLASS_TO_SWITCH[BUTTON_THRESHOLD_COUNT] = {
	3, 0, 1, 2, 4
};

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
#define RS485_TX_PIN     5
#define RS485_DE_RE_PIN  6
#define RS485_BAUD       2400   // common Pelco default - change to match the pan-tilt unit's DIP setting
#define RS485_ADDRESS    1      // pan-tilt unit's Pelco address (its DIP switches 1-6)

#define PELCO_HOME_PRESET 1     // preset number the pan-tilt head calls on "HOME"

// Home button (S5) actions:
//   short press (released within HOME_MAX_MS)               -> Home
//   CLEAR_HOME_PRESS_COUNT short presses in a row, each
//     starting within HOME_MULTI_PRESS_GAP_MS of the last
//     release                                                -> Clear Home
//     (the 1st press still runs Home; presses 2..N-1 only count)
//   released within [SAVE_HOME_MIN_MS, SAVE_HOME_MAX_MS]    -> Save Home
//   released in the gap between Home and Save Home (e.g. ~3s), or after
//     SAVE_HOME_MAX_MS -> nothing (buffer zone against mis-timed releases)
// Home/Save Home bands are deliberately wide - a human has no on-screen
// countdown while holding, so a 1-2s target window (the original design)
// was unhittable in practice.
#define HOME_MAX_MS             2000
#define SAVE_HOME_MIN_MS        4000
#define SAVE_HOME_MAX_MS        15000
#define CLEAR_HOME_PRESS_COUNT  5
#define HOME_MULTI_PRESS_GAP_MS 1000

// How long a "HOME" / "SAVE HOME" / "CLEAR HOME" confirmation stays on the
// TFT's status row after being triggered (by button or web UI).
#define STATUS_MESSAGE_MS  5000

// Pelco-D has no "move by N degrees" command - only "move until stopped".
// The head does answer a non-standard position-query extension (see
// queryPositionDeg() in Pelco.h), used to correct drift after each move,
// but there's still no "move to N degrees" command - each button press is
// turned into a fixed axis step by sending a move command at the
// current jog speed (adjustable via the web UI, see jogSpeed in main.cpp)
// for a computed duration, then stopping: duration scales with jog speed
// since the head's angular rate is assumed proportional to the speed byte.
// PELCO_MAX_PAN/TILT_SPEED_DEG_S are the unit's rated max speed (14 deg/s
// pan, 2 deg/s tilt at speed byte 63, per its manual) - re-measure against
// a known angle and adjust if the actual step size drifts.
#define DEFAULT_PAN_STEP_DEG       2.0f
#define DEFAULT_TILT_STEP_DEG      1.0f
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
#define AUTO_MAX_PULSE_DEG          30.0f  // longest single go-to pulse when far from the target

// Far from the target, the go-to drive "cruises": one continuous move at
// jogSpeed, polling the position every AUTO_CRUISE_POLL_MS, and stops when
// the remaining distance is under the braking distance - the distance the
// head covers in AUTO_BRAKE_LEAD_S at the current speed (never less than
// AUTO_SLOWDOWN_THRESHOLD_DEG). The short creep pulses above then finish
// the approach. Raise AUTO_BRAKE_LEAD_S if it overshoots, lower it if the
// final creep takes too long.
#define AUTO_CRUISE_POLL_MS         100
#define AUTO_BRAKE_LEAD_S           0.4f
#define AUTO_SETTLE_MS              300    // wait after stopping a cruise before re-reading position
#define AUTO_STALL_MS               1500   // no movement this long while cruising -> stop

// ---------- Pan/tilt travel limits (fixed) ----------
// Mechanical range of this head, in raw head-reported degrees (same frame
// queryPositionDeg() reads in). Measured once with the old Auto Calibrate -
// fixed for this unit. Pan can't pass the 350-360/0 dead zone, so Go to
// Azimut takes the long way round when the short way would cross it.
#define PAN_LIMIT_MIN_DEG       0.0f
#define PAN_LIMIT_MAX_DEG       350.0f
#define TILT_LIMIT_MIN_DEG      0.0f
#define TILT_LIMIT_MAX_DEG      50.0f

// Fixed position-target buttons in the web UI: drive straight to a known
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
