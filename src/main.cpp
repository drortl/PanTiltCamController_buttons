// Pan/Tilt camera controller — ESP32
// Pan/tilt motion is handled by an external self-contained RS485 pan-tilt
// head (own internal motor + MCU, Pelco-D/Pelco-P auto-adapting) - this
// board does not drive any local motors. The web UI's direction buttons
// step it by a fixed PELCO_STEP_DEG per press over Pelco-D/RS485 (see
// handleStep() and PelcoController in Pelco.h).
// 1.8" TFT (ST7735 128x160) status display + digital compass (QMC5883P)
//
// Pins used (full table + reserved-GPIO notes in config.h):
//   Signal            GPIO   Notes
//   -----------------------------------------------------
//   RS485 RX (RO)     4      UART2, Pelco-D - from the pan-tilt unit
//   RS485 TX (DI)     5      UART2, Pelco-D - to the pan-tilt unit
//   RS485 DE+/RE      6      module's DE and /RE pins tied together here
//   Compass SDA       7      QMC5883P, I2C addr 0x2C
//   Compass SCL       15
//   TFT BLK           16     backlight, driven HIGH = on
//   TFT CS            17     ST7735, 128x160
//   TFT DC            18
//   TFT RST           8
//   Button ADC        9      five-button resistor-ladder pad
//   TFT SDA           11     module's SPI data pin; hw SPI (ESP32-S3 default pin)
//   TFT SCL           12     module's SPI clock pin; hw SPI (ESP32-S3 default pin)
//
//   All on header J1, grouped by peripheral (RS485 -> Compass -> TFT) for
//   easy soldering - see config.h for the full physical-layout rationale.

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SPI.h>
#include <Preferences.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include "config.h"
#include "Qmc5883p.h"
#include "Pelco.h"
#include "web_page.h"

Adafruit_ST7735 tft(TFT_CS_PIN, TFT_DC_PIN, TFT_RST_PIN);
Qmc5883p compass;
WebServer server(80);
PelcoController pelco;
Preferences prefs;

// Forward declarations - doHome()/doSaveHome()/doClearHome() (defined near
// the HTTP handlers) put a confirmation on the TFT status row via
// showStatusMessage() (defined near updateDisplay()); updatePhysicalButtons()
// (defined above both) calls all of them for the Home button's press-duration
// handling.
void showStatusMessage(const char *msg);
void doHome();
void doSaveHome();
void doClearHome();

int lastButton = -1;
int candidateButton = -1;
uint8_t candidateCount = 0;
unsigned long lastButtonChangeMs = 0;
bool buttonMonitorEnabled = false;
unsigned long lastButtonMonitorMs = 0;

// Calibrated pin voltage (eFuse ADC calibration), averaged over 64 samples.
// Raw analogRead() counts read low near 0V, which squeezed the S4/S1 gap.
int readButtonMilliVolts() {
    uint32_t total = 0;
    for (int sample = 0; sample < 64; sample++) {
        total += analogReadMilliVolts(BUTTON_ADC_PIN);
    }
    return total / 64;
}

int readButton() {
    int milliVolts = readButtonMilliVolts();
    for (int button = 0; button < BUTTON_THRESHOLD_COUNT; button++) {
        if (milliVolts < BUTTON_MV_THRESHOLDS[button]) {
            return BUTTON_CLASS_TO_SWITCH[button];
        }
    }
    return -1;
}

bool compassOk = false;
bool wifiUsingFallbackAP = false;
String ipStr;
String wifiLabel; // SSID connected to, or "AP mode" if using the fallback

bool homeAzimuthSet = false;
float homeAzimuth = 0.0f;

// Pan/tilt position, tracked in exact PELCO_STEP_DEG increments (see
// handleStep() - each press moves by exactly one step, timed rather than
// estimated from an arbitrary hold duration). 0/0 means "at home"; becomes
// stale/unknown after recalling a non-home preset, since we don't know that
// preset's position relative to home. Periodically resynced against the
// head's own reported position (see correctPanDrift()/correctTiltDrift()
// below) to cancel drift from steps the head silently rounds or misses.
float panPositionDeg = 0.0f, tiltPositionDeg = 0.0f;
bool positionKnown = false; // true once Home/Save Home establishes a 0/0 reference

// The head's own raw queryPositionDeg() reading at the moment Save Home was
// last confirmed - the reference the drift correction below measures
// against. Only known if that query succeeded; correction is skipped until
// then (this unit is confirmed to answer it, but stay defensive - a swapped
// unit or bus glitch could mean no reply).
float panRawAtHome = 0.0f, tiltRawAtHome = 0.0f;
bool panRawAtHomeKnown = false, tiltRawAtHomeKnown = false;

// Both axes are back at the saved home position - resets the position
// reference to 0/0.
void resetPositionToHome() {
    panPositionDeg = 0.0f;
    tiltPositionDeg = 0.0f;
    positionKnown = true;
}

// Resyncs panPositionDeg/tiltPositionDeg against the head's real position,
// correcting for drift accumulated since the last correction. No-op if the
// raw-at-home reference isn't known yet, position is already flagged
// unknown (e.g. after recalling a non-home preset), or the head doesn't
// answer this time (transient bus noise) - dead reckoning is left as-is
// rather than showing a corrupted jump.
// Motor's own raw reported position (whatever queryPositionDeg() last read),
// independent of home/positionKnown - shown in the UI as "actual" position
// so it's useful right after Auto Calibrate, before Home is ever saved.
float rawPanDeg = 0, rawTiltDeg = 0;
bool rawPanKnown = false, rawTiltKnown = false;

void correctPanDrift() {
    if (!panRawAtHomeKnown || !positionKnown) return;
    float raw;
    if (!pelco.queryPositionDeg(RS485_ADDRESS, true, raw)) return;
    rawPanDeg = raw;
    rawPanKnown = true;
    // Pan wraps at 360 on the head's own reading; panPositionDeg does not
    // (it's a cumulative, possibly multi-turn count) - so compare against
    // the wrapped expected value and nudge by the shortest-path error,
    // preserving the turn count instead of snapping to it.
    float expected = fmodf(panRawAtHome + panPositionDeg, 360.0f);
    if (expected < 0) expected += 360.0f;
    float error = fmodf(raw - expected + 540.0f, 360.0f) - 180.0f;
    panPositionDeg += error;
}

void correctTiltDrift() {
    if (!tiltRawAtHomeKnown || !positionKnown) return;
    float raw;
    if (!pelco.queryPositionDeg(RS485_ADDRESS, false, raw)) return;
    rawTiltDeg = raw;
    rawTiltKnown = true;
    float expected = tiltRawAtHome + tiltPositionDeg; // tilt has no wraparound (limited mechanical range)
    tiltPositionDeg += (raw - expected);
}

// ---------- Jog speed (adjustable via web UI, persisted) ----------
uint8_t jogSpeed = PELCO_STEP_SPEED;
float panStepDeg = DEFAULT_PAN_STEP_DEG;
float tiltStepDeg = DEFAULT_TILT_STEP_DEG;

// Pulse duration for covering distDeg at speed, assuming the head's angular
// rate is proportional to the speed byte. speed-fraction floored so a very
// low speed setting doesn't turn a pulse into an impractically long block.
// Duration floored separately at MIN_PULSE_MS so a very small distDeg (the
// closed-loop drives' final creep pulses, see updateGoTo()) doesn't round
// down to a pulse so short the head's own motor start/stop lag swallows it
// entirely and it doesn't move at all. Shared by the fixed-step jog buttons
// (panStepMs()/tiltStepMs(), always a full PELCO_STEP_DEG at jogSpeed) and
// the closed-loop drives' creep pulses.
#define MIN_PULSE_MS 40
uint32_t stepMsFor(float maxSpeedDegS, float distDeg, uint8_t speed) {
    float frac = speed / 63.0f;
    if (frac < 0.08f) frac = 0.08f;
    uint32_t ms = (uint32_t)(1000.0f * distDeg / (maxSpeedDegS * frac));
    return ms < MIN_PULSE_MS ? MIN_PULSE_MS : ms;
}
uint32_t panStepMs() { return stepMsFor(PELCO_MAX_PAN_SPEED_DEG_S, panStepDeg, jogSpeed); }
uint32_t tiltStepMs() { return stepMsFor(PELCO_MAX_TILT_SPEED_DEG_S, tiltStepDeg, jogSpeed); }

// ---------- Pan/tilt travel limits (from Auto Calibrate or manual capture) ----------
// Stored as raw head-reported degrees (same frame queryPositionDeg() reads
// in) - not the home-relative panPositionDeg/tiltPositionDeg - so they can
// be compared directly against a fresh or estimated raw reading.
bool panLimitMinKnown = false, panLimitMaxKnown = false;
bool tiltLimitMinKnown = false, tiltLimitMaxKnown = false;
float panLimitMinDeg = 0, panLimitMaxDeg = 0;
float tiltLimitMinDeg = 0, tiltLimitMaxDeg = 0;

void saveLimitsToPrefs() {
    prefs.begin("pantilt", false);
    if (panLimitMinKnown) prefs.putFloat("panMin", panLimitMinDeg); else prefs.remove("panMin");
    if (panLimitMaxKnown) prefs.putFloat("panMax", panLimitMaxDeg); else prefs.remove("panMax");
    if (tiltLimitMinKnown) prefs.putFloat("tiltMin", tiltLimitMinDeg); else prefs.remove("tiltMin");
    if (tiltLimitMaxKnown) prefs.putFloat("tiltMax", tiltLimitMaxDeg); else prefs.remove("tiltMax");
    prefs.end();
}

// Estimated current raw position, from the home-relative dead-reckoned
// position plus the raw-at-home reference (same math correctPanDrift()/
// correctTiltDrift() use) - avoids an extra RS485 query on every step.
// Returns false (can't check) if there's no raw-at-home reference yet, i.e.
// home hasn't been saved - limits are only enforced once that exists.
bool estimatePanRaw(float &raw) {
    if (!panRawAtHomeKnown) return false;
    raw = panRawAtHome + panPositionDeg;
    return true;
}
bool estimateTiltRaw(float &raw) {
    if (!tiltRawAtHomeKnown) return false;
    raw = tiltRawAtHome + tiltPositionDeg;
    return true;
}

// dir > 0 = pan right / tilt up, dir < 0 = pan left / tilt down.
bool panAtLimit(int dir) {
    float raw;
    if (!estimatePanRaw(raw)) return false;
    if (dir > 0 && panLimitMaxKnown && raw >= panLimitMaxDeg) return true;
    if (dir < 0 && panLimitMinKnown && raw <= panLimitMinDeg) return true;
    return false;
}
bool tiltAtLimit(int dir) {
    float raw;
    if (!estimateTiltRaw(raw)) return false;
    if (dir > 0 && tiltLimitMaxKnown && raw >= tiltLimitMaxDeg) return true;
    if (dir < 0 && tiltLimitMinKnown && raw <= tiltLimitMinDeg) return true;
    return false;
}

// ---------- Auto Calibrate ----------
// Non-blocking state machine (ticked from loop(), see updateAutoCalibrate())
// that scans pan to one limit, pan to the other, tilt to one limit, tilt to
// the other - each phase detected by real position going flat/stalled
// rather than any fixed timing. Mirrors TickCalibratePhase() in the sibling
// USB app's MainForm.cs.
enum CalPhase { CAL_IDLE, CAL_PAN_MIN, CAL_PAN_MAX, CAL_TILT_MIN, CAL_TILT_MAX };
CalPhase calPhase = CAL_IDLE;
unsigned long calWindowStartMs = 0;
float calWindowStartDeg = 0;
int calFailCount = 0;
unsigned long calDeadlineMs = 0;
String calMessage = "Not calibrated";

// The "positive" scan direction (tiltUp/panRight bit) is assumed to
// increase the raw reading, but that's never actually verified against the
// hardware - if it's backward on a given unit, the "min" phase (which
// drives the OTHER bit) ends up stalling at the numerically larger raw
// value, leaving min > max. That inversion makes panAtLimit()/tiltAtLimit()
// block movement almost everywhere, since a raw value can be simultaneously
// "past max" and "past min". Normalize after each pair completes so min is
// always <= max regardless of which physical direction produced which
// value - also re-applied to whatever was last saved to flash, in case a
// previous calibration was already inverted.
void normalizeLimits(bool &minKnown, float &minDeg, bool &maxKnown, float &maxDeg) {
    if (minKnown && maxKnown && minDeg > maxDeg) {
        float t = minDeg; minDeg = maxDeg; maxDeg = t;
    }
}

void sendCalibrateMove(bool pan, bool positive) {
    bool panLeft = pan && !positive;
    bool panRight = pan && positive;
    bool tiltDown = !pan && !positive;
    bool tiltUp = !pan && positive;
    pelco.sendMove(RS485_ADDRESS, panLeft, panRight, tiltUp, tiltDown, CAL_SPEED, CAL_SPEED);
}

void advanceCalPhase(CalPhase next) {
    calWindowStartMs = 0;
    calFailCount = 0;
    calPhase = next;
    calDeadlineMs = millis() + CAL_PHASE_TIMEOUT_MS;
    if (next == CAL_PAN_MAX) calMessage = "Calibrating pan...";
    else if (next == CAL_TILT_MIN) calMessage = "Calibrating tilt...";
    else if (next == CAL_TILT_MAX) calMessage = "Calibrating tilt...";
}

void stopGoTo(); // defined below, alongside the rest of Go-to-position

void startAutoCalibrate() {
    stopGoTo();
    panLimitMinKnown = panLimitMaxKnown = tiltLimitMinKnown = tiltLimitMaxKnown = false;
    calWindowStartMs = 0;
    calFailCount = 0;
    calPhase = CAL_PAN_MIN;
    calDeadlineMs = millis() + CAL_PHASE_TIMEOUT_MS;
    calMessage = "Calibrating pan...";
}

void tickCalibratePhase(bool pan, bool positive, CalPhase nextPhase) {
    float deg;
    bool ok = pelco.queryPositionDeg(RS485_ADDRESS, pan, deg);
    if (!ok) {
        calFailCount++;
        if (calFailCount >= 5) {
            pelco.sendStop(RS485_ADDRESS);
            calMessage = String("Calibration failed - no ") + (pan ? "pan" : "tilt") + " reply";
            calPhase = CAL_IDLE;
        }
        return; // transient - keep the current move running, retry next tick
    }
    calFailCount = 0;
    if (pan) { rawPanDeg = deg; rawPanKnown = true; } else { rawTiltDeg = deg; rawTiltKnown = true; }

    if (calWindowStartMs == 0) {
        calWindowStartMs = millis();
        calWindowStartDeg = deg;
        sendCalibrateMove(pan, positive);
        return;
    }

    if (millis() - calWindowStartMs >= CAL_STALL_WINDOW_MS) {
        if (fabs(deg - calWindowStartDeg) < CAL_STALL_EPSILON_DEG) {
            pelco.sendStop(RS485_ADDRESS);
            if (pan) {
                if (!positive) { panLimitMinDeg = deg; panLimitMinKnown = true; } else { panLimitMaxDeg = deg; panLimitMaxKnown = true; }
                normalizeLimits(panLimitMinKnown, panLimitMinDeg, panLimitMaxKnown, panLimitMaxDeg);
            } else {
                if (!positive) { tiltLimitMinDeg = deg; tiltLimitMinKnown = true; } else { tiltLimitMaxDeg = deg; tiltLimitMaxKnown = true; }
                normalizeLimits(tiltLimitMinKnown, tiltLimitMinDeg, tiltLimitMaxKnown, tiltLimitMaxDeg);
            }
            if (nextPhase == CAL_IDLE) {
                saveLimitsToPrefs();
                calMessage = "Calibration complete";
            }
            advanceCalPhase(nextPhase);
            return;
        }
        calWindowStartMs = millis();
        calWindowStartDeg = deg;
    }
    // Move was already sent once when this phase started, and the head
    // keeps moving on its own until told to stop - no need to resend every
    // tick (that just adds RS485 traffic that can collide with the query
    // reply while the motor's drawing peak current).
}

void updateAutoCalibrate() {
    if (calPhase == CAL_IDLE) return;
    if ((long)(millis() - calDeadlineMs) > 0) {
        pelco.sendStop(RS485_ADDRESS);
        calMessage = "Calibration timed out";
        calPhase = CAL_IDLE;
        return;
    }
    switch (calPhase) {
        case CAL_PAN_MIN:  tickCalibratePhase(true,  false, CAL_PAN_MAX);  break;
        case CAL_PAN_MAX:  tickCalibratePhase(true,  true,  CAL_TILT_MIN); break;
        case CAL_TILT_MIN: tickCalibratePhase(false, false, CAL_TILT_MAX); break;
        case CAL_TILT_MAX: tickCalibratePhase(false, true,  CAL_IDLE);     break;
        default: break;
    }
}

// Before home is saved: Azimut tracks the live compass reading (for aiming
// the handheld controller/camera together during calibration).
// After home is saved: the compass is irrelevant (it stays with the
// portable controller, not the camera) - Azimut freezes relative to the
// compass and is instead driven purely by pan step position, wrapping at
// 0-360. Right increases it, left decreases it.
bool azimuthAvailable() {
    return homeAzimuthSet || compassOk;
}

float currentAzimuth() {
    if (homeAzimuthSet) {
        float az = fmodf(homeAzimuth + panPositionDeg, 360.0f);
        if (az < 0) az += 360.0f;
        return az;
    }
    int16_t cx, cy, cz;
    if (compassOk && compass.read(cx, cy, cz)) {
        return compass.headingFromRaw(cx, cy);
    }
    return 0.0f;
}

// ---------- Go-to-position (Go to Azimut, Pan Mid, Tilt Zero) ----------
// Drives one axis straight to a known raw motor degree value - either a
// fixed calibration target (PAN_MID_TARGET_DEG/TILT_ZERO_TARGET_DEG in
// config.h) or, for "Go to Azimut", a raw pan target computed from a fresh
// query at request time (see handleGoAzimuth()) so it always drives off the
// head's real current position rather than the dead-reckoned estimate.
// Non-blocking (pulse, wait, requery) so /status polling can report
// progress live and the web UI can flip the Go button from red to green on
// arrival - a single request that blocked until arrival could mean a very
// long wait for a large turn, with no feedback in the meantime. Only one
// axis drives at a time - the Pelco-D protocol carries pan and tilt bits in
// a single frame, so two independent movers could stomp on each other's
// commands. goSource exists only so /status can tell the web UI which
// button's "driving/arrived" indicator to update.
enum GoAxis { GO_NONE, GO_PAN, GO_TILT };
enum GoSource { GO_SRC_NONE, GO_SRC_AZIMUTH, GO_SRC_PAN_MID, GO_SRC_TILT_ZERO };
GoAxis goToAxis = GO_NONE;
GoSource goSource = GO_SRC_NONE;
float goToTargetDeg = 0;
bool goToPulsing = false;
unsigned long goToPulseUntilMs = 0;
unsigned long goToDeadlineMs = 0;

// Which physical command bit actually increases the raw reading is assumed
// (panRight/tiltUp = +1), but never verified - if that assumption is
// backward on a given unit, driving off it would push the head away from
// the target forever instead of converging. These track the *learned*
// correction per axis (starts at +1 = "assumption holds", flips to -1 if a
// pulse is ever seen to make the gap to target worse instead of better) and
// persist across drives once learned, same approach as _panDirSign/
// _tiltDirSign in the sibling USB app's homing loop (MainForm.cs).
int panDirSign = 1, tiltDirSign = 1;
float goToLastDiff = 0;
bool goToHasLastDiff = false;

// Periodic refresh of the raw motor position display (rawPanDeg/rawTiltDeg),
// for when nothing else (a step, calibration, go-to) is already querying
// it. Skipped while those are active so it doesn't add competing RS485
// traffic during a steering-critical sequence - see correctPanDrift()/
// correctTiltDrift() and tickCalibratePhase() for the other updaters.
unsigned long lastRawQueryMs = 0;
void refreshRawPosition() {
    if (calPhase != CAL_IDLE || goToAxis != GO_NONE) return;
    if (millis() - lastRawQueryMs < 500) return;
    lastRawQueryMs = millis();
    float deg;
    if (pelco.queryPositionDeg(RS485_ADDRESS, true, deg)) { rawPanDeg = deg; rawPanKnown = true; }
    if (pelco.queryPositionDeg(RS485_ADDRESS, false, deg)) { rawTiltDeg = deg; rawTiltKnown = true; }
}

void stopGoTo() {
    if (goToPulsing) pelco.sendStop(RS485_ADDRESS);
    goToAxis = GO_NONE;
    goSource = GO_SRC_NONE;
    goToPulsing = false;
}

void startGoTo(bool pan, float targetDeg, GoSource source) {
    stopGoTo();
    goToAxis = pan ? GO_PAN : GO_TILT;
    goSource = source;
    goToTargetDeg = targetDeg;
    goToDeadlineMs = millis() + GOTO_DRIVE_TIMEOUT_MS;
    goToHasLastDiff = false; // panDirSign/tiltDirSign deliberately NOT reset - once learned, stays learned
}

void updateGoTo() {
    if (goToAxis == GO_NONE) return;
    if ((long)(millis() - goToDeadlineMs) > 0) {
        pelco.sendStop(RS485_ADDRESS);
        goToAxis = GO_NONE;
        goSource = GO_SRC_NONE;
        return;
    }

    if (goToPulsing) {
        if ((long)(millis() - goToPulseUntilMs) < 0) return; // pulse still in flight
        pelco.sendStop(RS485_ADDRESS);
        goToPulsing = false;
    }

    bool pan = goToAxis == GO_PAN;
    float raw;
    if (!pelco.queryPositionDeg(RS485_ADDRESS, pan, raw)) return; // transient - retry next tick
    if (pan) { rawPanDeg = raw; rawPanKnown = true; } else { rawTiltDeg = raw; rawTiltKnown = true; }

    float diff = goToTargetDeg - raw;
    if (fabs(diff) < AUTO_ARRIVE_TOLERANCE_DEG) {
        goToAxis = GO_NONE; // arrived
        goSource = GO_SRC_NONE;
        if (pan) correctPanDrift(); else correctTiltDrift();
        return;
    }

    // If the last pulse made the gap to target worse instead of better, the
    // panRight/tiltUp-increases-raw assumption is backward for this axis on
    // this unit - flip and remember the correction (see panDirSign/
    // tiltDirSign above) rather than continuing to drive away from target.
    if (goToHasLastDiff && fabs(diff) > fabs(goToLastDiff) + 0.3f) {
        if (pan) panDirSign = -panDirSign; else tiltDirSign = -tiltDirSign;
    }
    goToLastDiff = diff;
    goToHasLastDiff = true;

    // dir: which way the RAW value needs to move (+1 = increase). Limit
    // checks and the panPositionDeg/tiltPositionDeg estimate are in that
    // same raw-moves-with-dir convention used throughout the rest of the
    // firmware (see correctPanDrift()/handleStep()). bitDir: which physical
    // command bit actually achieves that, after applying the learned sign.
    int dir = diff > 0 ? 1 : -1;
    if ((pan && panAtLimit(dir)) || (!pan && tiltAtLimit(dir))) {
        goToAxis = GO_NONE; // can't get closer without crossing a calibrated limit
        goSource = GO_SRC_NONE;
        return;
    }
    int bitDir = dir * (pan ? panDirSign : tiltDirSign);

    // Full-speed, full-length pulses would only ever land within half a
    // PELCO_STEP_DEG of the target (there's no smaller final-correction
    // pulse otherwise) - so slow to a shorter, slower "creep" pulse once
    // close, clamped to the actual remaining distance so the last pulse
    // can't overshoot past the target.
    bool creeping = fabs(diff) < AUTO_SLOWDOWN_THRESHOLD_DEG;
    float stepDist = creeping ? fminf(AUTO_CREEP_STEP_DEG, fabs(diff)) : pan ? panStepDeg : tiltStepDeg;
    uint8_t speed = creeping ? AUTO_CREEP_SPEED : jogSpeed;

    if (pan) {
        pelco.sendMove(RS485_ADDRESS, bitDir < 0, bitDir > 0, false, false, speed, speed);
        panPositionDeg += dir * stepDist;
        goToPulseUntilMs = millis() + stepMsFor(PELCO_MAX_PAN_SPEED_DEG_S, stepDist, speed);
    } else {
        pelco.sendMove(RS485_ADDRESS, false, false, bitDir > 0, bitDir < 0, speed, speed);
        tiltPositionDeg += dir * stepDist;
        goToPulseUntilMs = millis() + stepMsFor(PELCO_MAX_TILT_SPEED_DEG_S, stepDist, speed);
    }
    goToPulsing = true;
}

bool connectToWifi(const char *ssid, const char *password, unsigned long timeoutMs) {
    Serial.printf("Connecting to WiFi \"%s\"", ssid);
    WiFi.begin(ssid, password);
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) {
        delay(250);
        Serial.print(".");
    }
    Serial.println();
    return WiFi.status() == WL_CONNECTED;
}

void setupWifi() {
    WiFi.mode(WIFI_STA);
    if (connectToWifi(WIFI_SSID_PRIMARY, WIFI_PASSWORD_PRIMARY, WIFI_CONNECT_TIMEOUT_MS)) {
        wifiLabel = WIFI_SSID_PRIMARY;
    } else if (connectToWifi(WIFI_SSID_BACKUP, WIFI_PASSWORD_BACKUP, WIFI_CONNECT_TIMEOUT_MS)) {
        wifiLabel = WIFI_SSID_BACKUP;
    } else {
        Serial.println("Could not join either WiFi network - starting fallback access point");
        wifiUsingFallbackAP = true;
        WiFi.mode(WIFI_AP);
        WiFi.softAP(AP_SSID, AP_PASSWORD);
        ipStr = WiFi.softAPIP().toString();
        wifiLabel = "AP mode";
        return;
    }
    ipStr = WiFi.localIP().toString();
    Serial.printf("Connected to \"%s\", IP: %s\n", wifiLabel.c_str(), ipStr.c_str());
}

// STA connections can drop (out of range, router reboot, etc.) in a way an
// ESP32-hosted AP never does - check periodically and try to rejoin.
void maintainWifi() {
    if (wifiUsingFallbackAP) return;
    static unsigned long lastCheckMs = 0;
    if (WiFi.status() == WL_CONNECTED || millis() - lastCheckMs < 30000) return;
    lastCheckMs = millis();
    Serial.println("WiFi disconnected, reconnecting...");
    WiFi.reconnect();
}

void handleRoot() {
    server.send_P(200, "text/html", INDEX_HTML);
}

// Each button press moves by a fixed PELCO_STEP_DEG (5 degrees): send a
// move command at the current jog speed, block for the duration that takes
// at that speed, then stop. Blocking is intentional and simplest here -
// the physical head genuinely needs that long to complete the step, and a
// single button press has nothing else useful to do meanwhile.
void handleStep() {
    stopGoTo(); // manual jog overrides any in-progress closed-loop drive

    String axis = server.arg("axis");
    int dir = server.arg("dir").toInt() >= 0 ? 1 : -1;

    if (axis == "pan") {
        if (panAtLimit(dir)) { server.send(409, "text/plain", "limit"); return; }
        pelco.sendMove(RS485_ADDRESS, dir < 0, dir > 0, false, false, jogSpeed, jogSpeed);
        delay(panStepMs());
        pelco.sendStop(RS485_ADDRESS);
        panPositionDeg += dir * panStepDeg;
        correctPanDrift();
    } else if (axis == "tilt") {
        if (tiltAtLimit(dir)) { server.send(409, "text/plain", "limit"); return; }
        pelco.sendMove(RS485_ADDRESS, false, false, dir > 0, dir < 0, jogSpeed, jogSpeed);
        delay(tiltStepMs());
        pelco.sendStop(RS485_ADDRESS);
        tiltPositionDeg += dir * tiltStepDeg;
        correctTiltDrift();
    } else {
        server.send(400, "text/plain", "bad axis");
        return;
    }
    server.send(200, "text/plain", "ok");
}

void handlePhysicalButton(int button) {
    Serial.printf("Button S%d pressed\n", button + 1);
    if (button == 0) {
        pelco.sendMove(RS485_ADDRESS, false, false, true, false, jogSpeed, jogSpeed);
        delay(tiltStepMs());
        pelco.sendStop(RS485_ADDRESS);
        tiltPositionDeg += tiltStepDeg;
        correctTiltDrift();
    } else if (button == 1) {
        pelco.sendMove(RS485_ADDRESS, false, false, false, true, jogSpeed, jogSpeed);
        delay(tiltStepMs());
        pelco.sendStop(RS485_ADDRESS);
        tiltPositionDeg -= tiltStepDeg;
        correctTiltDrift();
    } else if (button == 2) {
        pelco.sendMove(RS485_ADDRESS, false, true, false, false, jogSpeed, jogSpeed);
        delay(panStepMs());
        pelco.sendStop(RS485_ADDRESS);
        panPositionDeg += panStepDeg;
        correctPanDrift();
    } else if (button == 3) {
        pelco.sendMove(RS485_ADDRESS, true, false, false, false, jogSpeed, jogSpeed);
        delay(panStepMs());
        pelco.sendStop(RS485_ADDRESS);
        panPositionDeg -= panStepDeg;
        correctPanDrift();
    }
    // button == 4 (Home, S5) is handled separately in updatePhysicalButtons()
    // by hold duration, not dispatched here.
}

bool homeHeld = false;
unsigned long homePressStartMs = 0;
bool homeClearFired = false; // Clear Home already fired for this hold (fires immediately, doesn't wait for release)

// Home (S5) release-time dispatch: total hold duration decides the action.
// Only reached if Clear Home didn't already fire mid-hold (see
// updatePhysicalButtons()). Bands are exact and exclusive - a release in the
// gap between them (e.g. ~3s) intentionally does nothing.
void dispatchHomeRelease(unsigned long heldMs) {
    if (heldMs <= HOME_MAX_MS) {
        doHome();
    } else if (heldMs >= SAVE_HOME_MIN_MS && heldMs <= SAVE_HOME_MAX_MS) {
        doSaveHome();
    }
}

void updatePhysicalButtons() {
    if (buttonMonitorEnabled) return;
    int button = readButton();
    if (button != candidateButton) {
        candidateButton = button;
        candidateCount = 1;
    } else if (candidateCount < 4) {
        candidateCount++;
    }
    bool stable = candidateCount >= 4;
    int stableButton = stable ? candidateButton : lastButton;

    // Home (S5) hold tracking - checked every call while held (not just on
    // the press/release edges) so Clear Home can fire the instant its
    // threshold is reached, without waiting for the button to be released.
    if (stableButton == 4) {
        if (!homeHeld) {
            homeHeld = true;
            homePressStartMs = millis();
            homeClearFired = false;
        } else if (!homeClearFired && millis() - homePressStartMs >= CLEAR_HOME_MIN_MS) {
            homeClearFired = true;
            doClearHome();
        }
    } else if (homeHeld) {
        // Home button just released.
        homeHeld = false;
        if (!homeClearFired) {
            dispatchHomeRelease(millis() - homePressStartMs);
        }
    }

    if (stableButton == lastButton) return;
    lastButtonChangeMs = millis();
    lastButton = stableButton;
    if (stableButton >= 0 && stableButton != 4) {
        stopGoTo();
        handlePhysicalButton(stableButton);
    }
}

void updateButtonSerialMonitor() {
    while (Serial.available()) {
        char command = (char)Serial.read();
        if (command == 'a' || command == 'A') {
            buttonMonitorEnabled = !buttonMonitorEnabled;
            lastButtonMonitorMs = 0;
            Serial.println(buttonMonitorEnabled
                ? "Button monitor ON - press S1-S5 and record mV values"
                : "Button monitor OFF - button control resumed");
        }
    }

    if (!buttonMonitorEnabled || millis() - lastButtonMonitorMs < 300) return;
    lastButtonMonitorMs = millis();
    int milliVolts = readButtonMilliVolts();
    int button = readButton();
    Serial.printf("button_mv=%d detected=%s\n", milliVolts,
                  button >= 0 ? String("S") + (button + 1) : "none");
}

// Safety/manual use (e.g. testing via curl) - the head already stops itself
// at the end of each step in handleStep(), so the web UI doesn't call this
// directly, though it also cancels any in-progress closed-loop drive.
void handleStop() {
    stopGoTo();
    pelco.sendStop(RS485_ADDRESS);
    server.send(200, "text/plain", "ok");
}

// Drives pan automatically toward a target Azimut - see updateGoTo() in
// loop(). Requires home to already be set, since "Azimut" before that is
// just the live (and camera-irrelevant) compass reading. Converts the
// target into a raw pan degree target using a fresh position query taken
// right now, rather than the dead-reckoned panPositionDeg estimate - Azimut
// and the head's raw reading move together 1:1 (they differ only by a
// fixed offset), so the shortest-path delta in Azimut space is exactly the
// delta needed in raw space too. Driving off a fresh reading like this
// means the result can't inherit any drift accumulated since the last
// correction, unlike the old dead-reckoned pulse-counting approach.
void handleGoAzimuth() {
    if (!homeAzimuthSet) { server.send(409, "text/plain", "home not set"); return; }
    float t = fmodf(server.arg("target").toFloat(), 360.0f);
    if (t < 0) t += 360.0f;

    float currentRaw;
    if (!pelco.queryPositionDeg(RS485_ADDRESS, true, currentRaw)) {
        server.send(504, "text/plain", "no reply");
        return;
    }
    rawPanDeg = currentRaw;
    rawPanKnown = true;

    // Shortest signed distance from the current Azimut to the target, in (-180, 180].
    float diff = fmodf(t - currentAzimuth() + 540.0f, 360.0f) - 180.0f;
    startGoTo(true, currentRaw + diff, GO_SRC_AZIMUTH);
    server.send(200, "text/plain", "ok");
}

// Drives pan straight to its calibrated center (PAN_MID_TARGET_DEG) - see
// updateGoTo() in loop().
void handleGoToPanMid() {
    startGoTo(true, PAN_MID_TARGET_DEG, GO_SRC_PAN_MID);
    server.send(200, "text/plain", "ok");
}

// Drives tilt straight to its level/zero position (TILT_ZERO_TARGET_DEG).
void handleGoToTiltZero() {
    startGoTo(false, TILT_ZERO_TARGET_DEG, GO_SRC_TILT_ZERO);
    server.send(200, "text/plain", "ok");
}

void doHome() {
    stopGoTo();
    pelco.callPreset(RS485_ADDRESS, PELCO_HOME_PRESET);
    resetPositionToHome(); // best-effort - assumes the head lands exactly at home
    showStatusMessage("HOME");
}

// Point the head where "home" should be (using the jog buttons), then call
// this: it tells the pan-tilt unit to remember the current position as its
// HOME preset, and saves the current compass heading as the home azimuth
// reference (persisted to flash) - the Azimut display then reads relative
// to this direction (0 = facing home) instead of raw compass heading.
void doSaveHome() {
    stopGoTo();
    pelco.setPreset(RS485_ADDRESS, PELCO_HOME_PRESET);
    resetPositionToHome();

    // Capture the head's own raw position reading right now as the
    // drift-correction reference - safe here (unlike doHome()) because
    // the head hasn't moved: this command just tells it to remember where
    // it already is, so "current raw position" and "home" are the same
    // point at this instant.
    float raw;
    panRawAtHomeKnown = pelco.queryPositionDeg(RS485_ADDRESS, true, raw);
    if (panRawAtHomeKnown) panRawAtHome = raw;
    tiltRawAtHomeKnown = pelco.queryPositionDeg(RS485_ADDRESS, false, raw);
    if (tiltRawAtHomeKnown) tiltRawAtHome = raw;

    int16_t cx, cy, cz;
    if (compassOk && compass.read(cx, cy, cz)) {
        homeAzimuth = compass.headingFromRaw(cx, cy);
        homeAzimuthSet = true;
        prefs.begin("pantilt", false);
        prefs.putFloat("homeAz", homeAzimuth);
        prefs.end();
    }
    showStatusMessage("SAVE HOME");
}

// Reverts to live compass tracking - undoes Save Home (including the
// persisted flash value), so Azimut goes back to following the compass
// live instead of being frozen/dead-reckoned.
void doClearHome() {
    stopGoTo();
    homeAzimuthSet = false;
    homeAzimuth = 0.0f;
    prefs.begin("pantilt", false);
    prefs.remove("homeAz");
    prefs.end();

    positionKnown = false;
    panPositionDeg = 0.0f;
    tiltPositionDeg = 0.0f;
    panRawAtHomeKnown = false;
    tiltRawAtHomeKnown = false;
    showStatusMessage("CLEAR HOME");
}

void handleHome() {
    doHome();
    server.send(200, "text/plain", "ok");
}

void handleSaveHome() {
    doSaveHome();
    server.send(200, "text/plain", "ok");
}

void handleClearHome() {
    doClearHome();
    server.send(200, "text/plain", "ok");
}

bool parsePresetNum(uint8_t &num) {
    int n = server.arg("num").toInt();
    if (n < 0 || n > 255) return false;
    num = (uint8_t)n;
    return true;
}

// ---------- User preset Azimut/tilt info (for web UI display only) ----------
// What Azimut/tilt each of the 3 user preset slots (PELCO_USER_PRESET_BASE
// .. +PELCO_USER_PRESET_COUNT-1) was saved at, so the web UI can show
// "(Az 190.5, Tilt +3.2)" next to each preset button. Captured at Save time
// and persisted to flash - purely informational, doesn't affect Go/Save
// behavior (which still goes through the head's own preset memory).
float presetAz[PELCO_USER_PRESET_COUNT] = {0};
float presetTilt[PELCO_USER_PRESET_COUNT] = {0};
bool presetAzKnown[PELCO_USER_PRESET_COUNT] = {false};
bool presetTiltKnown[PELCO_USER_PRESET_COUNT] = {false};

void savePresetInfoToPrefs(int idx) {
    char keyAz[8], keyTilt[8];
    snprintf(keyAz, sizeof(keyAz), "pAz%d", idx);
    snprintf(keyTilt, sizeof(keyTilt), "pTi%d", idx);
    prefs.begin("pantilt", false);
    if (presetAzKnown[idx]) prefs.putFloat(keyAz, presetAz[idx]); else prefs.remove(keyAz);
    if (presetTiltKnown[idx]) prefs.putFloat(keyTilt, presetTilt[idx]); else prefs.remove(keyTilt);
    prefs.end();
}

void handlePresetSet() {
    uint8_t num;
    if (!parsePresetNum(num)) { server.send(400, "text/plain", "bad preset"); return; }
    pelco.setPreset(RS485_ADDRESS, num);

    int idx = (int)num - PELCO_USER_PRESET_BASE;
    if (idx >= 0 && idx < PELCO_USER_PRESET_COUNT) {
        presetAzKnown[idx] = azimuthAvailable();
        if (presetAzKnown[idx]) presetAz[idx] = currentAzimuth();

        float raw;
        presetTiltKnown[idx] = pelco.queryPositionDeg(RS485_ADDRESS, false, raw);
        if (presetTiltKnown[idx]) {
            rawTiltDeg = raw;
            rawTiltKnown = true;
            presetTilt[idx] = raw - TILT_ZERO_TARGET_DEG; // relative to horizontal (see TILT_ZERO_TARGET_DEG)
        }
        savePresetInfoToPrefs(idx);
    }
    server.send(200, "text/plain", "ok");
}

void handlePresetGo() {
    uint8_t num;
    if (!parsePresetNum(num)) { server.send(400, "text/plain", "bad preset"); return; }
    stopGoTo();
    pelco.callPreset(RS485_ADDRESS, num);
    if (num == PELCO_HOME_PRESET) {
        resetPositionToHome();
    } else {
        // We don't know where this preset physically is relative to home -
        // mark position unknown rather than show a stale/wrong estimate.
        positionKnown = false;
    }
    server.send(200, "text/plain", "ok");
}

// Only relevant if another controller (e.g. a joystick) shares this RS485
// bus - not needed for the web UI, which drives the head directly via
// handleStep()/handleHome() above.
void handlePelcoCommand(const PelcoCommand &cmd) {
    (void)cmd;
}

void handleSetSpeed() {
    int v = server.arg("value").toInt();
    if (v < 1 || v > 63) { server.send(400, "text/plain", "bad speed"); return; }
    jogSpeed = (uint8_t)v;
    prefs.begin("pantilt", false);
    prefs.putUChar("speed", jogSpeed);
    prefs.end();
    server.send(200, "text/plain", "ok");
}

void handleSetSteps() {
    float newPanStep = server.arg("pan").toFloat();
    float newTiltStep = server.arg("tilt").toFloat();
    if (newPanStep < 0.1f || newPanStep > 20.0f ||
        newTiltStep < 0.1f || newTiltStep > 20.0f) {
        server.send(400, "text/plain", "bad step");
        return;
    }
    panStepDeg = newPanStep;
    tiltStepDeg = newTiltStep;
    prefs.begin("pantilt", false);
    prefs.putFloat("panStep", panStepDeg);
    prefs.putFloat("tiltStep", tiltStepDeg);
    prefs.end();
    server.send(200, "text/plain", "ok");
}

void handleAutoCalibrate() {
    startAutoCalibrate();
    server.send(200, "text/plain", "ok");
}

void handleCancelCalibrate() {
    if (calPhase != CAL_IDLE) {
        pelco.sendStop(RS485_ADDRESS);
        calPhase = CAL_IDLE;
        calMessage = "Cancelled";
    }
    server.send(200, "text/plain", "ok");
}

// Manual alternative to Auto Calibrate: jog to a mechanical limit with the
// direction buttons, then capture it here - useful if the automatic stall
// scan isn't reliable for a given axis.
void handleCalLimit() {
    bool pan = server.arg("axis") == "pan";
    bool isMin = server.arg("which") == "min";
    float raw;
    if (!pelco.queryPositionDeg(RS485_ADDRESS, pan, raw)) {
        server.send(504, "text/plain", "no reply");
        return;
    }
    if (pan) { if (isMin) { panLimitMinDeg = raw; panLimitMinKnown = true; } else { panLimitMaxDeg = raw; panLimitMaxKnown = true; } }
    else { if (isMin) { tiltLimitMinDeg = raw; tiltLimitMinKnown = true; } else { tiltLimitMaxDeg = raw; tiltLimitMaxKnown = true; } }
    saveLimitsToPrefs();
    server.send(200, "text/plain", "ok");
}

void handleStatus() {
    bool azOk = azimuthAvailable();
    float heading = azOk ? currentAzimuth() : 0.0f;

    char buf[900];
    int n = snprintf(buf, sizeof(buf),
        "{\"heading\":%.1f,\"compassOk\":%s,\"homeSet\":%s,"
        "\"pan\":%.1f,\"tilt\":%.1f,\"posKnown\":%s,\"autoDrive\":%s,"
        "\"rawPanOk\":%s,\"rawPan\":%.1f,\"rawTiltOk\":%s,\"rawTilt\":%.1f,"
        "\"tiltRelOk\":%s,\"tiltRel\":%.1f,"
        "\"speed\":%u,"
        "\"panStep\":%.1f,\"tiltStep\":%.1f,"
        "\"calActive\":%s,\"calMsg\":\"%s\","
        "\"panMinOk\":%s,\"panMin\":%.1f,\"panMaxOk\":%s,\"panMax\":%.1f,"
        "\"tiltMinOk\":%s,\"tiltMin\":%.1f,\"tiltMaxOk\":%s,\"tiltMax\":%.1f,"
        "\"goToPan\":%s,\"goToTilt\":%s,"
        "\"presets\":[",
        heading, azOk ? "true" : "false", homeAzimuthSet ? "true" : "false",
        panPositionDeg, tiltPositionDeg, positionKnown ? "true" : "false",
        (goToAxis == GO_PAN && goSource == GO_SRC_AZIMUTH) ? "true" : "false",
        rawPanKnown ? "true" : "false", rawPanDeg, rawTiltKnown ? "true" : "false", rawTiltDeg,
        rawTiltKnown ? "true" : "false", rawTiltDeg - TILT_ZERO_TARGET_DEG,
        jogSpeed,
        panStepDeg, tiltStepDeg,
        calPhase != CAL_IDLE ? "true" : "false", calMessage.c_str(),
        panLimitMinKnown ? "true" : "false", panLimitMinDeg,
        panLimitMaxKnown ? "true" : "false", panLimitMaxDeg,
        tiltLimitMinKnown ? "true" : "false", tiltLimitMinDeg,
        tiltLimitMaxKnown ? "true" : "false", tiltLimitMaxDeg,
        (goToAxis == GO_PAN && goSource == GO_SRC_PAN_MID) ? "true" : "false",
        (goToAxis == GO_TILT && goSource == GO_SRC_TILT_ZERO) ? "true" : "false");
    if (n < 0) n = 0;
    if (n >= (int)sizeof(buf)) n = sizeof(buf) - 1;

    for (int i = 0; i < PELCO_USER_PRESET_COUNT && n < (int)sizeof(buf) - 1; i++) {
        int w = snprintf(buf + n, sizeof(buf) - n,
            "%s{\"azOk\":%s,\"az\":%.1f,\"tiltOk\":%s,\"tilt\":%.1f}",
            i == 0 ? "" : ",",
            presetAzKnown[i] ? "true" : "false", presetAz[i],
            presetTiltKnown[i] ? "true" : "false", presetTilt[i]);
        if (w < 0) break;
        n += w;
        if (n >= (int)sizeof(buf) - 1) { n = sizeof(buf) - 1; break; }
    }
    n += snprintf(buf + n, sizeof(buf) - n, "]}");
    server.send(200, "application/json", buf);
}

// Confirmation shown on the TFT's status row (below Azimut/Pan/Tilt) for
// STATUS_MESSAGE_MS after Home/Save Home/Clear Home fires, whether that came
// from the physical button or the web UI - see doHome()/doSaveHome()/doClearHome().
String statusMessage = "";
unsigned long statusMessageUntilMs = 0;

void showStatusMessage(const char *msg) {
    statusMessage = msg;
    statusMessageUntilMs = millis() + STATUS_MESSAGE_MS;
}

void drawStaticScreen() {
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(1);
    tft.setCursor(4, 4);
    tft.println(wifiLabel + ":");
    tft.setCursor(4, 14);
    tft.println(ipStr);
    tft.drawFastHLine(0, 26, tft.width(), ST77XX_CYAN);
}

void updateDisplay() {
    static float lastHeading = -9999, lastPan = -9999, lastTilt = -9999;
    static unsigned long lastUpdateMs = 0;
    static String shownStatus = "";

    if (millis() - lastUpdateMs < 300) return; // cap TFT refresh rate
    lastUpdateMs = millis();

    bool azOk = azimuthAvailable();
    float heading = azOk ? currentAzimuth() : lastHeading;
    float pan = panPositionDeg;
    float tilt = tiltPositionDeg;

    // While the Home button is held past HOME_MAX_MS, show a live "HOLD Ns"
    // counter so the user can see how long they've held it (helps them land
    // in the Save Home window / reach Clear Home). Once Clear Home has fired
    // for this hold, its confirmation message takes over immediately instead
    // (see doClearHome()/updatePhysicalButtons()) rather than waiting on release.
    bool holdCountdownActive = homeHeld && !homeClearFired && (millis() - homePressStartMs) > HOME_MAX_MS;
    unsigned long holdSec = holdCountdownActive ? (millis() - homePressStartMs) / 1000 : 0;

    bool statusActive = statusMessage.length() > 0 && (long)(millis() - statusMessageUntilMs) < 0;
    String statusToShow;
    if (holdCountdownActive) {
        char buf[16];
        snprintf(buf, sizeof(buf), "HOLD %lus", holdSec);
        statusToShow = buf;
    } else {
        statusToShow = statusActive ? statusMessage : "";
    }

    if (fabs(heading - lastHeading) < 0.5f && fabs(pan - lastPan) < 0.1f && fabs(tilt - lastTilt) < 0.1f
        && statusToShow == shownStatus) {
        return; // nothing meaningfully changed, skip redraw to avoid flicker
    }
    lastHeading = heading;
    lastPan = pan;
    lastTilt = tilt;
    bool statusChanged = statusToShow != shownStatus;
    shownStatus = statusToShow;

    // Text is drawn with a black background and padded to a fixed width, so
    // each line overwrites its old pixels in place. Clearing the area first
    // (fillRect) took ~60ms at 4MHz SPI and showed as a visible black flash.
    char line[16];
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    tft.setCursor(4, 38);
    if (azOk) {
        snprintf(line, sizeof(line), "Azimut %6.1f", heading);
    } else {
        snprintf(line, sizeof(line), "NO COMPASS");
    }
    tft.printf("%-13s", line);

    tft.setTextColor(positionKnown ? ST77XX_YELLOW : ST77XX_WHITE, ST77XX_BLACK);
    tft.setCursor(4, 58);
    if (positionKnown) {
        snprintf(line, sizeof(line), "Pan  %6.1f", pan);
    } else {
        snprintf(line, sizeof(line), "Pan     ??");
    }
    tft.printf("%-13s", line);
    tft.setCursor(4, 78);
    if (positionKnown) {
        snprintf(line, sizeof(line), "Tilt %6.1f", tilt);
    } else {
        snprintf(line, sizeof(line), "Tilt    ??");
    }
    tft.printf("%-13s", line);

    // The status line can wrap, so it is cleared only when its text changes.
    if (statusChanged) {
        tft.fillRect(0, 98, tft.width(), tft.height() - 98, ST77XX_BLACK);
        if (statusToShow.length() > 0) {
            tft.setTextColor(holdCountdownActive ? ST77XX_YELLOW : ST77XX_GREEN, ST77XX_BLACK);
            tft.setCursor(4, 98);
            tft.print(statusToShow);
        }
    }
}

// Diagnostic only - if the compass ever stops being found, this narrows
// down whether it's wiring/power vs. a wrong I2C address - check the
// Serial Monitor at boot. (Confirmed via hardware test: this module is a
// QMC5883P at 0x2C, despite GY-271 listings usually showing HMC5883L stock
// photos.)
void scanI2C() {
    Serial.println("Scanning I2C bus...");
    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("  found device at 0x%02X\n", addr);
            found++;
        }
    }
    if (found == 0) Serial.println("  no I2C devices found - check wiring/power");
}

void setup() {
    Serial.begin(115200);

    analogReadResolution(10);
    analogSetPinAttenuation(BUTTON_ADC_PIN, ADC_11db);
    pinMode(BUTTON_ADC_PIN, INPUT);
    Serial.println("Send 'a' in Serial Monitor to start/stop button ADC monitor");

    pinMode(TFT_BLK_PIN, OUTPUT);
    digitalWrite(TFT_BLK_PIN, HIGH); // backlight on

    // Lowered from the 16MHz default - random colorful static that never
    // resolves into an image is the classic symptom of SPI signal integrity
    // problems on breadboard jumper wires; a slower clock is more tolerant
    // of that. Raise this back up once wiring is confirmed solid.
    tft.setSPISpeed(4000000);
    tft.initR(INITR_BLACKTAB); // switch to INITR_GREENTAB if colors look wrong
    tft.setRotation(1);        // landscape, 160x128

    pelco.begin(Serial2, RS485_RX_PIN, RS485_TX_PIN, RS485_DE_RE_PIN, RS485_BAUD);

    // Runs right after RS485/LCD, before WiFi - a stuck or miswired I2C bus
    // (compass not ACKing, or SDA/SCL shorted) can only ever cost the
    // compass reading, so it doesn't need to wait behind the WiFi connect
    // timeout (up to 20s) to report status on Serial.
    compassOk = compass.begin();
    if (!compassOk) {
        Serial.println("QMC5883P not found - check wiring (SDA=21/SCL=14) and I2C address 0x2C");
        scanI2C();
    }

    prefs.begin("pantilt", true);
    homeAzimuthSet = prefs.isKey("homeAz");
    homeAzimuth = prefs.getFloat("homeAz", 0.0f);
    jogSpeed = prefs.getUChar("speed", PELCO_STEP_SPEED);
    panStepDeg = prefs.getFloat("panStep", DEFAULT_PAN_STEP_DEG);
    tiltStepDeg = prefs.getFloat("tiltStep", DEFAULT_TILT_STEP_DEG);
    panLimitMinKnown = prefs.isKey("panMin");  panLimitMinDeg  = prefs.getFloat("panMin", 0.0f);
    panLimitMaxKnown = prefs.isKey("panMax");  panLimitMaxDeg  = prefs.getFloat("panMax", 0.0f);
    tiltLimitMinKnown = prefs.isKey("tiltMin"); tiltLimitMinDeg = prefs.getFloat("tiltMin", 0.0f);
    tiltLimitMaxKnown = prefs.isKey("tiltMax"); tiltLimitMaxDeg = prefs.getFloat("tiltMax", 0.0f);
    // Fixes up a previously-saved inverted pair (min > max) from before
    // normalizeLimits() existed - see its comment for why that can happen.
    normalizeLimits(panLimitMinKnown, panLimitMinDeg, panLimitMaxKnown, panLimitMaxDeg);
    normalizeLimits(tiltLimitMinKnown, tiltLimitMinDeg, tiltLimitMaxKnown, tiltLimitMaxDeg);
    for (int i = 0; i < PELCO_USER_PRESET_COUNT; i++) {
        char keyAz[8], keyTilt[8];
        snprintf(keyAz, sizeof(keyAz), "pAz%d", i);
        snprintf(keyTilt, sizeof(keyTilt), "pTi%d", i);
        presetAzKnown[i] = prefs.isKey(keyAz);
        presetAz[i] = prefs.getFloat(keyAz, 0.0f);
        presetTiltKnown[i] = prefs.isKey(keyTilt);
        presetTilt[i] = prefs.getFloat(keyTilt, 0.0f);
    }
    prefs.end();

    setupWifi();
    drawStaticScreen();

    server.on("/", handleRoot);
    server.on("/move", handleStep);
    server.on("/stop", handleStop);
    server.on("/home", handleHome);
    server.on("/saveHome", handleSaveHome);
    server.on("/clearHome", handleClearHome);
    server.on("/goAzimuth", handleGoAzimuth);
    server.on("/goToPanMid", handleGoToPanMid);
    server.on("/goToTiltZero", handleGoToTiltZero);
    server.on("/presetSet", handlePresetSet);
    server.on("/presetGo", handlePresetGo);
    server.on("/setSpeed", handleSetSpeed);
        server.on("/setSteps", handleSetSteps);
    server.on("/autoCalibrate", handleAutoCalibrate);
    server.on("/cancelCalibrate", handleCancelCalibrate);
    server.on("/calLimit", handleCalLimit);
    server.on("/status", handleStatus);
    server.begin();
}

void loop() {
    server.handleClient();
    updateButtonSerialMonitor();
    updatePhysicalButtons();
    maintainWifi();
    updateGoTo();
    updateAutoCalibrate();
    refreshRawPosition();

    PelcoCommand pelcoCmd;
    if (pelco.poll(pelcoCmd)) {
        handlePelcoCommand(pelcoCmd);
    }

    updateDisplay();
}
