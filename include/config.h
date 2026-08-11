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
// TFT CS                   5      ST7735, 128x160
// TFT DC                   17
// TFT RST                  16
// TFT SCL  (hw SPI, fixed) 12     module's SPI clock pin; ESP32-S3 default SPI pin
// TFT SDA  (hw SPI, fixed) 11     module's SPI data pin; ESP32-S3 default SPI pin
// TFT BLK                  6      backlight, driven HIGH = on
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

// ---------- TFT (ST7735, 128x160, hardware SPI) ----------
#define TFT_CS_PIN      5
#define TFT_DC_PIN      17
#define TFT_RST_PIN     16
#define TFT_BLK_PIN     6
// SCL=12, SDA=11 (the module's labels for SPI clock/data - not I2C) are the
// ESP32-S3 default hardware SPI pins, used automatically by the library
// since no explicit SPI pins are passed.
// If colors look wrong/inverted, switch INITR_BLACKTAB to INITR_GREENTAB in main.cpp.

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

// Pelco-D has no "move by N degrees" command - only "move until stopped",
// with no position feedback. Each button press is turned into a fixed
// PELCO_STEP_DEG move by sending a move command at full speed (63) for a
// computed duration, then stopping: duration = PELCO_STEP_DEG / rated speed.
// PELCO_MAX_PAN/TILT_SPEED_DEG_S are the unit's rated max speed (14 deg/s
// pan, 2 deg/s tilt at speed byte 63, per its manual) - re-measure against
// a known angle and adjust if the actual step size drifts from 5 degrees.
#define PELCO_STEP_DEG             5.0f
#define PELCO_STEP_SPEED           63     // max Pelco-D speed byte (0-63) - used for step moves
#define PELCO_MAX_PAN_SPEED_DEG_S  14.0f
#define PELCO_MAX_TILT_SPEED_DEG_S 2.0f
#define PAN_STEP_MS  ((uint32_t)(1000.0f * PELCO_STEP_DEG / PELCO_MAX_PAN_SPEED_DEG_S))
#define TILT_STEP_MS ((uint32_t)(1000.0f * PELCO_STEP_DEG / PELCO_MAX_TILT_SPEED_DEG_S))

// "Go to Azimut" auto-drive: how close counts as "arrived" - half a step,
// since we can't land more precisely than one PELCO_STEP_DEG pulse anyway.
#define AZIMUTH_ARRIVE_TOLERANCE_DEG (PELCO_STEP_DEG / 2.0f)
