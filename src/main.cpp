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
#include <BLEDevice.h>
#include <BLEServer.h>
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
String wifiLabel; // SSID connected to, or AP_SSID when hosting the access point

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

// Set at boot when a raw-at-home reference was restored from flash - the
// first successful raw query in refreshRawPosition() then rebuilds
// panPositionDeg/tiltPositionDeg from the head's real position, replacing
// the last saved (possibly stale) position. Waits for the head to answer,
// since it may still be powering up after an outage.
bool positionSyncPending = false;

// Last position written to flash (see savePositionWhenSettled()).
float savedPanDeg = 0.0f, savedTiltDeg = 0.0f;
bool savedPositionKnown = false;

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
// so it's useful before Home is ever saved.
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

// ---------- Pan/tilt travel limits ----------
// Fixed in config.h (PAN_LIMIT_*/TILT_LIMIT_*), as raw head-reported
// degrees - not the home-relative panPositionDeg/tiltPositionDeg - so they
// can be compared directly against a fresh or estimated raw reading.

// Estimated current raw position, from the home-relative dead-reckoned
// position plus the raw-at-home reference (same math correctPanDrift()/
// correctTiltDrift() use) - avoids an extra RS485 query on every step.
// Returns false (can't check) if there's no raw-at-home reference yet, i.e.
// home hasn't been saved - limits are only enforced once that exists.
bool estimatePanRaw(float &raw) {
    if (!panRawAtHomeKnown) return false;
    // The head reports pan in [0, 360) but panPositionDeg is cumulative, so
    // wrap - otherwise e.g. home raw 350 + 20 = 370 reads as past any limit.
    raw = fmodf(panRawAtHome + panPositionDeg, 360.0f);
    if (raw < 0) raw += 360.0f;
    return true;
}
bool estimateTiltRaw(float &raw) {
    if (!tiltRawAtHomeKnown) return false;
    raw = tiltRawAtHome + tiltPositionDeg;
    return true;
}

// dir > 0 = pan right / tilt up, dir < 0 = pan left / tilt down.
// The *Raw versions check a real raw reading (used by the go-to drive, which
// has a fresh one); the others check the dead-reckoned estimate.
bool panAtLimitRaw(float raw, int dir) {
    if (dir > 0 && raw >= PAN_LIMIT_MAX_DEG) return true;
    if (dir < 0 && raw <= PAN_LIMIT_MIN_DEG) return true;
    return false;
}
bool tiltAtLimitRaw(float raw, int dir) {
    if (dir > 0 && raw >= TILT_LIMIT_MAX_DEG) return true;
    if (dir < 0 && raw <= TILT_LIMIT_MIN_DEG) return true;
    return false;
}
bool panAtLimit(int dir) {
    float raw;
    return estimatePanRaw(raw) && panAtLimitRaw(raw, dir);
}
bool tiltAtLimit(int dir) {
    float raw;
    return estimateTiltRaw(raw) && tiltAtLimitRaw(raw, dir);
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
// Far from the target it cruises (one continuous move, polling position
// while moving - see AUTO_CRUISE_POLL_MS in config.h) so long turns are one
// smooth motion; near the target it switches to short creep pulses.
// Non-blocking (pulse/poll, wait, requery) so /status polling can report
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

// Cruise state (continuous move, see updateGoTo()).
bool goToCruising = false;
int goToCruiseDir = 1;           // raw direction being driven (+1 = increase)
float goToCruiseBestDiff = 0;    // smallest |gap| seen this cruise (wrong-direction check)
float goToLastRaw = 0;
unsigned long goToNextPollMs = 0;
unsigned long goToLastMoveMs = 0;

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
// for when nothing else (a step, go-to) is already querying it. Skipped
// while a go-to runs so it doesn't add competing RS485 traffic during a
// steering-critical sequence - see correctPanDrift()/correctTiltDrift()
// and updateGoTo() for the other updaters.
unsigned long lastRawQueryMs = 0;
void refreshRawPosition() {
    if (goToAxis != GO_NONE) return;
    if (millis() - lastRawQueryMs < 500) return;
    lastRawQueryMs = millis();
    float deg;
    if (pelco.queryPositionDeg(RS485_ADDRESS, true, deg)) { rawPanDeg = deg; rawPanKnown = true; }
    if (pelco.queryPositionDeg(RS485_ADDRESS, false, deg)) { rawTiltDeg = deg; rawTiltKnown = true; }

    if (positionSyncPending && rawPanKnown && rawTiltKnown) {
        positionSyncPending = false;
        // Start from the saved position so pan keeps its turn count, then
        // nudge by the shortest-path error - same math as correctPanDrift().
        if (!positionKnown) { panPositionDeg = 0.0f; tiltPositionDeg = 0.0f; }
        float expected = fmodf(panRawAtHome + panPositionDeg, 360.0f);
        if (expected < 0) expected += 360.0f;
        panPositionDeg += fmodf(rawPanDeg - expected + 540.0f, 360.0f) - 180.0f;
        tiltPositionDeg = rawTiltDeg - tiltRawAtHome;
        positionKnown = true;
    }
}

// Writes the current pan/tilt position to flash once it has stopped
// changing for POSITION_SAVE_DELAY_MS, so it survives a power loss without
// a flash write on every step. Skipped while a go-to is moving the head.
#define POSITION_SAVE_DELAY_MS 2000
float lastSeenPanDeg = 0.0f, lastSeenTiltDeg = 0.0f;
bool lastSeenPositionKnown = false;
unsigned long lastPositionChangeMs = 0;
void savePositionWhenSettled() {
    if (panPositionDeg != lastSeenPanDeg || tiltPositionDeg != lastSeenTiltDeg ||
        positionKnown != lastSeenPositionKnown ||
        goToAxis != GO_NONE) {
        lastSeenPanDeg = panPositionDeg;
        lastSeenTiltDeg = tiltPositionDeg;
        lastSeenPositionKnown = positionKnown;
        lastPositionChangeMs = millis();
        return;
    }
    if (millis() - lastPositionChangeMs < POSITION_SAVE_DELAY_MS) return;
    if (positionKnown == savedPositionKnown &&
        fabsf(panPositionDeg - savedPanDeg) < 0.05f &&
        fabsf(tiltPositionDeg - savedTiltDeg) < 0.05f) return;

    prefs.begin("pantilt", false);
    prefs.putBool("posOk", positionKnown);
    prefs.putFloat("panPos", panPositionDeg);
    prefs.putFloat("tiltPos", tiltPositionDeg);
    prefs.end();
    savedPositionKnown = positionKnown;
    savedPanDeg = panPositionDeg;
    savedTiltDeg = tiltPositionDeg;
}

void stopGoTo() {
    if (goToPulsing || goToCruising) pelco.sendStop(RS485_ADDRESS);
    goToAxis = GO_NONE;
    goSource = GO_SRC_NONE;
    goToPulsing = false;
    goToCruising = false;
}

// Sends the continuous move for a cruise in raw direction dir, applying
// the learned panDirSign/tiltDirSign.
void sendCruiseMove(bool pan, int dir) {
    int bitDir = dir * (pan ? panDirSign : tiltDirSign);
    if (pan) pelco.sendMove(RS485_ADDRESS, bitDir < 0, bitDir > 0, false, false, jogSpeed, jogSpeed);
    else     pelco.sendMove(RS485_ADDRESS, false, false, bitDir > 0, bitDir < 0, jogSpeed, jogSpeed);
}

// Distance the head covers in AUTO_BRAKE_LEAD_S at jogSpeed - same speed
// model as stepMsFor().
float cruiseBrakeDeg(bool pan) {
    float frac = jogSpeed / 63.0f;
    if (frac < 0.08f) frac = 0.08f;
    float degS = (pan ? PELCO_MAX_PAN_SPEED_DEG_S : PELCO_MAX_TILT_SPEED_DEG_S) * frac;
    return fmaxf(degS * AUTO_BRAKE_LEAD_S, AUTO_SLOWDOWN_THRESHOLD_DEG);
}

// Ends a cruise: stop, then let the pulse logic in updateGoTo() finish the
// approach after AUTO_SETTLE_MS (reuses the pulse wait, so the head's
// coasting has ended before the next position read).
void endCruise() {
    pelco.sendStop(RS485_ADDRESS);
    goToCruising = false;
    goToHasLastDiff = false;
    goToPulsing = true;
    goToPulseUntilMs = millis() + AUTO_SETTLE_MS;
}

// One cruise tick: poll position while moving and decide when to stop.
void updateCruise() {
    if ((long)(millis() - goToNextPollMs) < 0) return;
    goToNextPollMs = millis() + AUTO_CRUISE_POLL_MS;

    bool pan = goToAxis == GO_PAN;
    float raw;
    if (!pelco.queryPositionDeg(RS485_ADDRESS, pan, raw)) return; // transient - keep moving, deadline/stall guard
    if (pan) { rawPanDeg = raw; rawPanKnown = true; } else { rawTiltDeg = raw; rawTiltKnown = true; }

    float moved = raw - goToLastRaw;
    goToLastRaw = raw;
    if (fabs(moved) > 0.05f) goToLastMoveMs = millis();
    if (pan) panPositionDeg += moved; else tiltPositionDeg += moved;

    float diff = goToTargetDeg - raw;
    int dir = goToCruiseDir;

    // Gap grew while still on the start side of the target -> the command
    // bit is backward for this axis. Flip, remember, and drive the other way.
    if (diff * dir > 0 && fabs(diff) > goToCruiseBestDiff + 0.5f) {
        Serial.printf("Go-to cruise: moving away from target (%.1f -> %.1f), flipping %s direction\n",
                      goToCruiseBestDiff * dir, diff, pan ? "pan" : "tilt");
        if (pan) panDirSign = -panDirSign; else tiltDirSign = -tiltDirSign;
        goToCruiseBestDiff = fabs(diff);
        goToLastMoveMs = millis();
        sendCruiseMove(pan, dir);
        return;
    }
    if (diff * dir > 0) goToCruiseBestDiff = fminf(goToCruiseBestDiff, fabs(diff));

    bool passed = diff * dir <= 0;
    bool nearTarget = fabs(diff) <= cruiseBrakeDeg(pan);
    bool atLimit = pan ? panAtLimitRaw(raw, dir) : tiltAtLimitRaw(raw, dir);
    bool stalled = millis() - goToLastMoveMs > AUTO_STALL_MS;
    if (stalled) {
        // Blocked (e.g. a mechanical stop) - give up rather than retrying
        // against it until GOTO_DRIVE_TIMEOUT_MS.
        Serial.printf("Go-to cruise: %s not moving at raw %.1f, giving up\n", pan ? "pan" : "tilt", raw);
        stopGoTo();
        return;
    }
    if (passed || nearTarget || atLimit) endCruise();
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
        goToPulsing = false;
        goToCruising = false;
        return;
    }

    if (goToCruising) {
        updateCruise();
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

    // No wraparound: the target is always inside the fixed limits (see
    // handleGoAzimuth()), so driving straight there never crosses the pan
    // dead zone between PAN_LIMIT_MAX_DEG and PAN_LIMIT_MIN_DEG.
    float diff = goToTargetDeg - raw;
    if (fabs(diff) < AUTO_ARRIVE_TOLERANCE_DEG) {
        goToAxis = GO_NONE; // arrived
        goSource = GO_SRC_NONE;
        if (pan) correctPanDrift(); else correctTiltDrift();
        return;
    }

    // If the last pulse moved the head AWAY from the target (gap grew, same
    // side of the target), the panRight/tiltUp-increases-raw assumption is
    // backward for this axis on this unit - flip and remember the correction
    // (see panDirSign/tiltDirSign above). An overshoot (gap changed sign)
    // moved the right way and must not flip - doing so made the drive turn
    // back and forth around the target.
    if (goToHasLastDiff && diff * goToLastDiff > 0 && fabs(diff) > fabs(goToLastDiff) + 0.3f) {
        Serial.printf("Go-to: moved away from target (%.1f -> %.1f), flipping %s direction\n",
                      goToLastDiff, diff, pan ? "pan" : "tilt");
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
    if ((pan && panAtLimitRaw(raw, dir)) || (!pan && tiltAtLimitRaw(raw, dir))) {
        Serial.printf("Go-to stopped at %s limit: raw %.1f, target %.1f\n", pan ? "pan" : "tilt", raw, goToTargetDeg);
        goToAxis = GO_NONE; // can't get closer without crossing a pan/tilt limit
        goSource = GO_SRC_NONE;
        return;
    }
    // Far away: one continuous move instead of pulses (see updateCruise()).
    if (fabs(diff) > cruiseBrakeDeg(pan)) {
        goToCruising = true;
        goToCruiseDir = dir;
        goToCruiseBestDiff = fabs(diff);
        goToLastRaw = raw;
        goToLastMoveMs = millis();
        goToNextPollMs = millis() + AUTO_CRUISE_POLL_MS;
        sendCruiseMove(pan, dir);
        return;
    }

    int bitDir = dir * (pan ? panDirSign : tiltDirSign);

    // Full-speed, full-length pulses would only ever land within half a
    // PELCO_STEP_DEG of the target (there's no smaller final-correction
    // pulse otherwise) - so slow to a shorter, slower "creep" pulse once
    // close, clamped to the actual remaining distance so the last pulse
    // can't overshoot past the target.
    // When far away, each pulse covers half the remaining gap (capped at
    // AUTO_MAX_PULSE_DEG) instead of one jog step - a 2 deg jog step needed
    // ~170 pulses for a long turn and could hit GOTO_DRIVE_TIMEOUT_MS. Half
    // the gap also means an error in the assumed head speed can't overshoot
    // past the target by more than the gap itself.
    bool creeping = fabs(diff) < AUTO_SLOWDOWN_THRESHOLD_DEG;
    float stepDist = creeping ? fminf(AUTO_CREEP_STEP_DEG, fabs(diff))
                              : fmaxf(fminf(fabs(diff) * 0.5f, AUTO_MAX_PULSE_DEG), AUTO_CREEP_STEP_DEG);
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

// WiFi settings, editable from the web UI and BLE and saved in flash
// ("staSsid"/"staPass"/"apSsid"/"apPass"); secrets.h / config.h only give
// the defaults. Passwords are never sent back to a client.
String staSsid = WIFI_SSID_PRIMARY, staPass = WIFI_PASSWORD_PRIMARY;
String apSsid = AP_SSID, apPass = AP_PASSWORD;

// WiFi on/off (BLE only - "wifi on" / "wifi off"), saved in flash
// ("wifiOn"). Off = radio off, control over Bluetooth only.
bool wifiEnabled = true;

String jsonEscape(const String &s) {
    String out;
    for (size_t i = 0; i < s.length(); i++) {
        char c = s[i];
        if (c == '"' || c == '\\') { out += '\\'; out += c; }
        else if ((uint8_t)c < 0x20) out += ' ';
        else out += c;
    }
    return out;
}

void startAccessPoint() {
    wifiUsingFallbackAP = true;
    WiFi.mode(WIFI_AP);
    WiFi.softAP(apSsid.c_str(), apPass.c_str());
    ipStr = WiFi.softAPIP().toString();
    wifiLabel = apSsid;
    Serial.printf("Access point \"%s\" started, IP: %s\n", apSsid.c_str(), ipStr.c_str());
}

// WiFi mode, switchable from the web UI / BLE ("wifi ap" / "wifi sta") and
// saved in flash ("apMode"); WIFI_AP_ONLY in config.h is only the default.
// true = host the access point, false = join the home network (falls back
// to the access point if neither network is reachable). A change is applied
// by restarting, scheduled via wifiRestartAtMs so the reply gets sent first.
bool wifiApMode = WIFI_AP_ONLY;
unsigned long wifiRestartAtMs = 0;

void setupWifi() {
    if (!wifiEnabled) {
        WiFi.mode(WIFI_OFF);
        wifiLabel = "WiFi off (BT only)";
        ipStr = "";
        Serial.println("WiFi is off - Bluetooth control only");
        return;
    }
    if (wifiApMode) {
        startAccessPoint();
        return;
    }
    WiFi.mode(WIFI_STA);
    // Saved home network first, then the backup from secrets.h.
    if (connectToWifi(staSsid.c_str(), staPass.c_str(), WIFI_CONNECT_TIMEOUT_MS)) {
        wifiLabel = staSsid;
    } else if (staSsid != WIFI_SSID_BACKUP &&
               connectToWifi(WIFI_SSID_BACKUP, WIFI_PASSWORD_BACKUP, WIFI_CONNECT_TIMEOUT_MS)) {
        wifiLabel = WIFI_SSID_BACKUP;
    } else {
        Serial.println("Could not join either WiFi network - starting fallback access point");
        startAccessPoint();
        return;
    }
    ipStr = WiFi.localIP().toString();
    Serial.printf("Connected to \"%s\", IP: %s\n", wifiLabel.c_str(), ipStr.c_str());
}

// STA connections can drop (out of range, router reboot, etc.) in a way an
// ESP32-hosted AP never does - check periodically and try to rejoin.
void maintainWifi() {
    if (!wifiEnabled || wifiUsingFallbackAP) return;
    static unsigned long lastCheckMs = 0;
    if (WiFi.status() == WL_CONNECTED || millis() - lastCheckMs < 30000) return;
    lastCheckMs = millis();
    Serial.println("WiFi disconnected, reconnecting...");
    WiFi.reconnect();
}

// WiFi changes are applied by restarting. Saves the current position first
// (so the restart doesn't lose it) and schedules the restart - see loop().
void scheduleWifiRestart(const char *tftMsg) {
    stopGoTo();
    prefs.begin("pantilt", false);
    prefs.putBool("posOk", positionKnown);
    prefs.putFloat("panPos", panPositionDeg);
    prefs.putFloat("tiltPos", tiltPositionDeg);
    prefs.end();
    wifiRestartAtMs = millis() + 1500;
    showStatusMessage(tftMsg);
    Serial.printf("%s - restarting\n", tftMsg);
}

void requestWifiMode(bool ap) {
    prefs.begin("pantilt", false);
    prefs.putBool("apMode", ap);
    prefs.end();
    scheduleWifiRestart(ap ? "WIFI: AP" : "WIFI: HOME");
}

void requestWifiEnabled(bool on) {
    prefs.begin("pantilt", false);
    prefs.putBool("wifiOn", on);
    prefs.end();
    scheduleWifiRestart(on ? "WIFI: ON" : "WIFI: OFF");
}

// Shared by the web UI and BLE. sta = home network, else access point.
// An empty password keeps the saved one. Returns false (msg = reason) if
// the values are not valid.
bool doSetWifiConfig(bool sta, String ssid, String pass, const char *&msg) {
    ssid.trim();
    if (ssid.length() < 1 || ssid.length() > 32) { msg = "name must be 1-32 characters"; return false; }
    if (pass.length() > 0 && (pass.length() < 8 || pass.length() > 63)) {
        msg = "password must be 8-63 characters"; return false;
    }
    prefs.begin("pantilt", false);
    if (sta) {
        staSsid = ssid;
        prefs.putString("staSsid", staSsid);
        if (pass.length() > 0) { staPass = pass; prefs.putString("staPass", staPass); }
    } else {
        apSsid = ssid;
        prefs.putString("apSsid", apSsid);
        if (pass.length() > 0) { apPass = pass; prefs.putString("apPass", apPass); }
    }
    prefs.end();
    scheduleWifiRestart(sta ? "HOME WIFI SAVED" : "AP SAVED");
    msg = "ok, restarting";
    return true;
}

// POST form fields: kind=sta|ap, ssid, pass (empty = keep).
void handleSetWifiConfig() {
    String kind = server.arg("kind");
    if (kind != "sta" && kind != "ap") { server.send(400, "text/plain", "bad kind"); return; }
    const char *msg;
    bool ok = doSetWifiConfig(kind == "sta", server.arg("ssid"), server.arg("pass"), msg);
    server.send(ok ? 200 : 400, "text/plain", msg);
}

void handleSetWifiMode() {
    String mode = server.arg("mode");
    if (mode != "ap" && mode != "sta") { server.send(400, "text/plain", "bad mode"); return; }
    requestWifiMode(mode == "ap");
    server.send(200, "text/plain", "ok");
}

void handleRoot() {
    server.send_P(200, "text/html", INDEX_HTML);
}

// Each button press moves by a fixed PELCO_STEP_DEG (5 degrees): send a
// move command at the current jog speed, block for the duration that takes
// at that speed, then stop. Blocking is intentional and simplest here -
// the physical head genuinely needs that long to complete the step, and a
// single button press has nothing else useful to do meanwhile.
// Shared by the web UI (handleStep()) and BLE (handleBleCommand()).
// Returns false if the axis is already at its limit in that direction.
bool doStep(bool pan, int dir) {
    stopGoTo(); // manual jog overrides any in-progress closed-loop drive
    if (pan) {
        if (panAtLimit(dir)) return false;
        pelco.sendMove(RS485_ADDRESS, dir < 0, dir > 0, false, false, jogSpeed, jogSpeed);
        delay(panStepMs());
        pelco.sendStop(RS485_ADDRESS);
        panPositionDeg += dir * panStepDeg;
        correctPanDrift();
    } else {
        if (tiltAtLimit(dir)) return false;
        pelco.sendMove(RS485_ADDRESS, false, false, dir > 0, dir < 0, jogSpeed, jogSpeed);
        delay(tiltStepMs());
        pelco.sendStop(RS485_ADDRESS);
        tiltPositionDeg += dir * tiltStepDeg;
        correctTiltDrift();
    }
    return true;
}

void handleStep() {
    String axis = server.arg("axis");
    int dir = server.arg("dir").toInt() >= 0 ? 1 : -1;
    if (axis != "pan" && axis != "tilt") { server.send(400, "text/plain", "bad axis"); return; }
    if (!doStep(axis == "pan", dir)) { server.send(409, "text/plain", "limit"); return; }
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

// Short-press counter for Clear Home (CLEAR_HOME_PRESS_COUNT presses in a
// row). Resets when the next press starts more than HOME_MULTI_PRESS_GAP_MS
// after the last short-press release, or on any long hold.
uint8_t homeShortPressCount = 0;
unsigned long homeLastReleaseMs = 0;

// Home (S5) release-time dispatch: total hold duration decides the action.
// Bands are exact and exclusive - a release in the gap between them
// (e.g. ~3s) intentionally does nothing.
void dispatchHomeRelease(unsigned long heldMs) {
    if (heldMs <= HOME_MAX_MS) {
        homeShortPressCount++;
        homeLastReleaseMs = millis();
        if (homeShortPressCount == 1) {
            doHome();
        } else if (homeShortPressCount >= CLEAR_HOME_PRESS_COUNT) {
            homeShortPressCount = 0;
            doClearHome();
        } else {
            char msg[16];
            snprintf(msg, sizeof(msg), "CLEAR %d/%d", homeShortPressCount, CLEAR_HOME_PRESS_COUNT);
            showStatusMessage(msg);
        }
        return;
    }
    homeShortPressCount = 0;
    if (heldMs >= SAVE_HOME_MIN_MS && heldMs <= SAVE_HOME_MAX_MS) {
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

    // Home (S5) press/release tracking - the action is decided on release
    // (see dispatchHomeRelease()).
    if (stableButton == 4) {
        if (!homeHeld) {
            homeHeld = true;
            homePressStartMs = millis();
            if (millis() - homeLastReleaseMs > HOME_MULTI_PRESS_GAP_MS) {
                homeShortPressCount = 0; // too slow - start a new sequence
            }
        }
    } else if (homeHeld) {
        // Home button just released.
        homeHeld = false;
        dispatchHomeRelease(millis() - homePressStartMs);
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
// Shared by the web UI and BLE. Returns the HTTP-style status code; msg is
// the reply text.
int doGoAzimuth(float t, const char *&msg) {
    if (!homeAzimuthSet) { msg = "home not set"; return 409; }
    t = fmodf(t, 360.0f);
    if (t < 0) t += 360.0f;

    float currentRaw;
    if (!pelco.queryPositionDeg(RS485_ADDRESS, true, currentRaw)) { msg = "no reply"; return 504; }
    rawPanDeg = currentRaw;
    rawPanKnown = true;

    // Shortest signed distance from the current Azimut to the target, in (-180, 180].
    float diff = fmodf(t - currentAzimuth() + 540.0f, 360.0f) - 180.0f;
    // Raw target in the head's own [0, 360) frame.
    float target = fmodf(currentRaw + diff, 360.0f);
    if (target < 0) target += 360.0f;
    // The head can't pass the dead zone between the limits - clamp so the
    // drive goes as far as it can instead of stopping.
    bool clamped = false;
    if (target < PAN_LIMIT_MIN_DEG) { target = PAN_LIMIT_MIN_DEG; clamped = true; }
    if (target > PAN_LIMIT_MAX_DEG) { target = PAN_LIMIT_MAX_DEG; clamped = true; }
    Serial.printf("Go to Azimut %.1f: azimut now %.1f, raw now %.1f, raw target %.1f%s\n",
                  t, currentAzimuth(), currentRaw, target, clamped ? " (clamped to limit)" : "");
    startGoTo(true, target, GO_SRC_AZIMUTH);
    msg = clamped ? "ok (limited by pan range)" : "ok";
    return 200;
}

void handleGoAzimuth() {
    const char *msg;
    int code = doGoAzimuth(server.arg("target").toFloat(), msg);
    server.send(code, "text/plain", msg);
}

// Drives pan straight to its measured center (PAN_MID_TARGET_DEG) - see
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
    positionSyncPending = false;

    prefs.begin("pantilt", false);
    if (panRawAtHomeKnown) prefs.putFloat("panRaw0", panRawAtHome); else prefs.remove("panRaw0");
    if (tiltRawAtHomeKnown) prefs.putFloat("tiltRaw0", tiltRawAtHome); else prefs.remove("tiltRaw0");
    prefs.end();

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
    prefs.remove("panRaw0");
    prefs.remove("tiltRaw0");
    prefs.end();

    positionKnown = false;
    panPositionDeg = 0.0f;
    tiltPositionDeg = 0.0f;
    panRawAtHomeKnown = false;
    tiltRawAtHomeKnown = false;
    positionSyncPending = false;
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

// Preset save/go and step sizes are shared by the web UI and BLE, so both
// show and change the same values.
void doPresetSave(uint8_t num) {
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
}

void doPresetGo(uint8_t num) {
    stopGoTo();
    pelco.callPreset(RS485_ADDRESS, num);
    if (num == PELCO_HOME_PRESET) {
        resetPositionToHome();
    } else {
        // We don't know where this preset physically is relative to home -
        // mark position unknown rather than show a stale/wrong estimate.
        positionKnown = false;
    }
}

void handlePresetSet() {
    uint8_t num;
    if (!parsePresetNum(num)) { server.send(400, "text/plain", "bad preset"); return; }
    doPresetSave(num);
    server.send(200, "text/plain", "ok");
}

void handlePresetGo() {
    uint8_t num;
    if (!parsePresetNum(num)) { server.send(400, "text/plain", "bad preset"); return; }
    doPresetGo(num);
    server.send(200, "text/plain", "ok");
}

// Returns false if a value is out of range (0.1-20 deg).
bool doSetSteps(float newPanStep, float newTiltStep) {
    if (newPanStep < 0.1f || newPanStep > 20.0f || newTiltStep < 0.1f || newTiltStep > 20.0f) return false;
    panStepDeg = newPanStep;
    tiltStepDeg = newTiltStep;
    prefs.begin("pantilt", false);
    prefs.putFloat("panStep", panStepDeg);
    prefs.putFloat("tiltStep", tiltStepDeg);
    prefs.end();
    return true;
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
    if (!doSetSteps(server.arg("pan").toFloat(), server.arg("tilt").toFloat())) {
        server.send(400, "text/plain", "bad step");
        return;
    }
    server.send(200, "text/plain", "ok");
}

void handleStatus() {
    bool azOk = azimuthAvailable();
    float heading = azOk ? currentAzimuth() : 0.0f;

    char buf[1200];
    int n = snprintf(buf, sizeof(buf),
        "{\"heading\":%.1f,\"compassOk\":%s,\"homeSet\":%s,"
        "\"pan\":%.1f,\"tilt\":%.1f,\"posKnown\":%s,\"autoDrive\":%s,"
        "\"rawPanOk\":%s,\"rawPan\":%.1f,\"rawTiltOk\":%s,\"rawTilt\":%.1f,"
        "\"tiltRelOk\":%s,\"tiltRel\":%.1f,"
        "\"speed\":%u,"
        "\"panStep\":%.1f,\"tiltStep\":%.1f,"
        "\"panMin\":%.1f,\"panMax\":%.1f,\"tiltMin\":%.1f,\"tiltMax\":%.1f,"
        "\"goToPan\":%s,\"goToTilt\":%s,"
        "\"apMode\":%s,\"wifiFallback\":%s,\"staSsid\":\"%s\",\"apSsid\":\"%s\","
        "\"presets\":[",
        heading, azOk ? "true" : "false", homeAzimuthSet ? "true" : "false",
        panPositionDeg, tiltPositionDeg, positionKnown ? "true" : "false",
        (goToAxis == GO_PAN && goSource == GO_SRC_AZIMUTH) ? "true" : "false",
        rawPanKnown ? "true" : "false", rawPanDeg, rawTiltKnown ? "true" : "false", rawTiltDeg,
        rawTiltKnown ? "true" : "false", rawTiltDeg - TILT_ZERO_TARGET_DEG,
        jogSpeed,
        panStepDeg, tiltStepDeg,
        PAN_LIMIT_MIN_DEG, PAN_LIMIT_MAX_DEG, TILT_LIMIT_MIN_DEG, TILT_LIMIT_MAX_DEG,
        (goToAxis == GO_PAN && goSource == GO_SRC_PAN_MID) ? "true" : "false",
        (goToAxis == GO_TILT && goSource == GO_SRC_TILT_ZERO) ? "true" : "false",
        wifiApMode ? "true" : "false",
        (!wifiApMode && wifiUsingFallbackAP) ? "true" : "false",
        jsonEscape(staSsid).c_str(), jsonEscape(apSsid).c_str());
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

// ---------- Bluetooth LE control (see ble/index.html, Web Bluetooth) ----------
// The ESP32-S3 only has BLE (no Classic Bluetooth), so the WiFi web page
// can't be served over it - instead a separate Web Bluetooth page writes
// short text commands to BLE_CMD_UUID and reads a compact JSON status from
// BLE_STATUS_UUID. Runs alongside WiFi.
//
// Commands (one per write): "step pan 1" / "step tilt -1" (dir: +1 = right/
// up), "stop", "home", "savehome", "clearhome", "goaz <deg>", "panmid",
// "tiltzero", "speed <1-63>", "steps <pan> <tilt>", "preset go|save <1-3>",
// "wifi ap|sta" (mode), "wifi on|off", and
// "wificfg sta|ap\n<ssid>\n<password>" (password empty = keep). The reply
// text is reported as "res" in the status (never the wificfg text, which
// holds the password). setMTU(247) lets the longer wificfg command fit in
// one write.
//
// BLE callbacks run on the BLE task, not loop() - so onWrite only queues the
// command, and loop() runs it (pelco/RS485 and all state stay single-threaded).
BLECharacteristic *bleStatusChar = nullptr;
portMUX_TYPE bleMux = portMUX_INITIALIZER_UNLOCKED;
char blePendingCmd[128] = "";
bool bleCmdPending = false;
bool bleConnected = false;
String bleLastResult = "";

class BleCmdCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *c) override {
        std::string v = c->getValue();
        portENTER_CRITICAL(&bleMux);
        strncpy(blePendingCmd, v.c_str(), sizeof(blePendingCmd) - 1);
        blePendingCmd[sizeof(blePendingCmd) - 1] = '\0';
        bleCmdPending = true;
        portEXIT_CRITICAL(&bleMux);
    }
};

class BleServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer *s) override { bleConnected = true; }
    void onDisconnect(BLEServer *s) override {
        bleConnected = false;
        BLEDevice::startAdvertising(); // let the next client connect
    }
};

void setupBle() {
    BLEDevice::init(BLE_DEVICE_NAME);
    BLEDevice::setPower(BLE_TX_POWER);
    BLEDevice::setMTU(247);
    BLEServer *srv = BLEDevice::createServer();
    srv->setCallbacks(new BleServerCallbacks());
    BLEService *svc = srv->createService(BLE_SERVICE_UUID);
    BLECharacteristic *cmd = svc->createCharacteristic(BLE_CMD_UUID,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
    cmd->setCallbacks(new BleCmdCallbacks());
    bleStatusChar = svc->createCharacteristic(BLE_STATUS_UUID, BLECharacteristic::PROPERTY_READ);
    bleStatusChar->setValue("{}");
    svc->start();
    BLEAdvertising *adv = BLEDevice::getAdvertising();
    adv->addServiceUUID(BLE_SERVICE_UUID);
    adv->setScanResponse(true);
    BLEDevice::startAdvertising();
    Serial.printf("BLE advertising as \"%s\"\n", BLE_DEVICE_NAME);
}

// "wificfg sta|ap\n<ssid>\n<password>" - handled apart from the other
// commands: names may contain spaces, and the text must not be logged or
// echoed since it holds the password.
void handleBleWifiConfig(const char *cmdText) {
    String text = cmdText;
    int nl1 = text.indexOf('\n');
    int nl2 = nl1 < 0 ? -1 : text.indexOf('\n', nl1 + 1);
    String kind = nl1 < 0 ? "" : text.substring(8, nl1);
    kind.trim();
    if (nl2 < 0 || (kind != "sta" && kind != "ap")) {
        bleLastResult = "wifi settings: bad format";
        return;
    }
    const char *msg;
    doSetWifiConfig(kind == "sta", text.substring(nl1 + 1, nl2), text.substring(nl2 + 1), msg);
    Serial.printf("BLE wifi settings (%s): %s\n", kind.c_str(), msg);
    bleLastResult = String(kind == "sta" ? "home wifi" : "access point") + ": " + msg;
}

void handleBleCommand(const char *cmdText) {
    if (!strncmp(cmdText, "wificfg ", 8)) { handleBleWifiConfig(cmdText); return; }
    char cmd[16] = "";
    char arg1[16] = "";
    float num = 0;
    int parsed = sscanf(cmdText, "%15s %15s %f", cmd, arg1, &num);
    Serial.printf("BLE command: \"%s\"\n", cmdText);
    String res = "ok";

    if (!strcmp(cmd, "step") && parsed == 3 && (!strcmp(arg1, "pan") || !strcmp(arg1, "tilt"))) {
        if (!doStep(!strcmp(arg1, "pan"), num >= 0 ? 1 : -1)) res = "limit";
    } else if (!strcmp(cmd, "stop")) {
        stopGoTo();
        pelco.sendStop(RS485_ADDRESS);
    } else if (!strcmp(cmd, "home")) {
        doHome();
    } else if (!strcmp(cmd, "savehome")) {
        doSaveHome();
    } else if (!strcmp(cmd, "clearhome")) {
        doClearHome();
    } else if (!strcmp(cmd, "goaz") && parsed >= 2) {
        const char *msg;
        doGoAzimuth(atof(arg1), msg);
        res = msg;
    } else if (!strcmp(cmd, "panmid")) {
        startGoTo(true, PAN_MID_TARGET_DEG, GO_SRC_PAN_MID);
    } else if (!strcmp(cmd, "tiltzero")) {
        startGoTo(false, TILT_ZERO_TARGET_DEG, GO_SRC_TILT_ZERO);
    } else if (!strcmp(cmd, "wifi") && parsed >= 2 && (!strcmp(arg1, "ap") || !strcmp(arg1, "sta"))) {
        requestWifiMode(!strcmp(arg1, "ap"));
        res = "ok, restarting";
    } else if (!strcmp(cmd, "wifi") && parsed >= 2 && (!strcmp(arg1, "on") || !strcmp(arg1, "off"))) {
        requestWifiEnabled(!strcmp(arg1, "on"));
        res = "ok, restarting";
    } else if (!strcmp(cmd, "preset") && parsed == 3 && (!strcmp(arg1, "go") || !strcmp(arg1, "save")) &&
               num >= 1 && num <= PELCO_USER_PRESET_COUNT) {
        uint8_t presetNum = PELCO_USER_PRESET_BASE + (int)num - 1; // slot 1..3 -> head preset number
        if (!strcmp(arg1, "go")) doPresetGo(presetNum); else doPresetSave(presetNum);
    } else if (!strcmp(cmd, "steps") && parsed == 3) {
        if (!doSetSteps(atof(arg1), num)) res = "step must be 0.1-20";
    } else if (!strcmp(cmd, "speed") && parsed >= 2 && atoi(arg1) >= 1 && atoi(arg1) <= 63) {
        jogSpeed = (uint8_t)atoi(arg1);
        prefs.begin("pantilt", false);
        prefs.putUChar("speed", jogSpeed);
        prefs.end();
    } else {
        res = "unknown command";
    }
    bleLastResult = String(cmdText) + ": " + res;
    bleLastResult.replace("\"", "'"); // keep the status JSON valid
    bleLastResult.replace("\\", "/");
}

// Called from loop(): runs a queued command and refreshes the status value.
void updateBle() {
    char cmd[sizeof(blePendingCmd)];
    bool have = false;
    portENTER_CRITICAL(&bleMux);
    if (bleCmdPending) {
        memcpy(cmd, blePendingCmd, sizeof(cmd));
        bleCmdPending = false;
        have = true;
    }
    portEXIT_CRITICAL(&bleMux);
    if (have) handleBleCommand(cmd);

    static unsigned long lastStatusMs = 0;
    if (!bleConnected || millis() - lastStatusMs < 250) return;
    lastStatusMs = millis();
    bool azOk = azimuthAvailable();
    // Presets as [azOk, az, tiltOk, tilt] per slot - same values the web UI shows.
    String presets = "[";
    for (int i = 0; i < PELCO_USER_PRESET_COUNT; i++) {
        char p[48];
        snprintf(p, sizeof(p), "%s[%d,%.1f,%d,%.1f]", i ? "," : "",
                 presetAzKnown[i], presetAz[i], presetTiltKnown[i], presetTilt[i]);
        presets += p;
    }
    presets += "]";
    // Keep the reply text short so the status stays under the 512-byte BLE limit.
    String res = bleLastResult.length() > 60 ? bleLastResult.substring(0, 60) : bleLastResult;

    char buf[512]; // max BLE attribute size; read with long reads, so the MTU doesn't limit it
    snprintf(buf, sizeof(buf),
        "{\"az\":%.1f,\"azOk\":%d,\"home\":%d,\"pan\":%.1f,\"tilt\":%.1f,\"pk\":%d,"
        "\"rp\":%.1f,\"rt\":%.1f,\"drv\":%d,\"spd\":%u,\"ps\":%.1f,\"ts\":%.1f,"
        "\"lim\":[%.0f,%.0f,%.0f,%.0f],\"pr\":%s,"
        "\"ap\":%d,\"wifi\":%d,\"fb\":%d,"
        "\"staSsid\":\"%s\",\"apSsid\":\"%s\",\"res\":\"%s\"}",
        azOk ? currentAzimuth() : 0.0f, azOk, homeAzimuthSet, panPositionDeg, tiltPositionDeg,
        positionKnown, rawPanDeg, rawTiltDeg - TILT_ZERO_TARGET_DEG, goToAxis != GO_NONE, jogSpeed,
        panStepDeg, tiltStepDeg,
        PAN_LIMIT_MIN_DEG, PAN_LIMIT_MAX_DEG, TILT_LIMIT_MIN_DEG, TILT_LIMIT_MAX_DEG, presets.c_str(),
        wifiApMode, wifiEnabled, wifiEnabled && !wifiApMode && wifiUsingFallbackAP,
        jsonEscape(staSsid).c_str(), jsonEscape(apSsid).c_str(), jsonEscape(res).c_str());
    bleStatusChar->setValue(buf);
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
    tft.println(ipStr.length() ? wifiLabel + ":" : wifiLabel);
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
    // in the Save Home window).
    bool holdCountdownActive = homeHeld && (millis() - homePressStartMs) > HOME_MAX_MS;
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
    wifiApMode = prefs.getBool("apMode", WIFI_AP_ONLY);
    wifiEnabled = prefs.getBool("wifiOn", true);
    staSsid = prefs.getString("staSsid", WIFI_SSID_PRIMARY);
    staPass = prefs.getString("staPass", WIFI_PASSWORD_PRIMARY);
    apSsid = prefs.getString("apSsid", AP_SSID);
    apPass = prefs.getString("apPass", AP_PASSWORD);
    panStepDeg = prefs.getFloat("panStep", DEFAULT_PAN_STEP_DEG);
    tiltStepDeg = prefs.getFloat("tiltStep", DEFAULT_TILT_STEP_DEG);
    for (int i = 0; i < PELCO_USER_PRESET_COUNT; i++) {
        char keyAz[8], keyTilt[8];
        snprintf(keyAz, sizeof(keyAz), "pAz%d", i);
        snprintf(keyTilt, sizeof(keyTilt), "pTi%d", i);
        presetAzKnown[i] = prefs.isKey(keyAz);
        presetAz[i] = prefs.getFloat(keyAz, 0.0f);
        presetTiltKnown[i] = prefs.isKey(keyTilt);
        presetTilt[i] = prefs.getFloat(keyTilt, 0.0f);
    }
    // Restore the last position from before power was lost. If the
    // raw-at-home reference is also saved, refreshRawPosition() corrects it
    // from the head's real position as soon as the head answers.
    panRawAtHomeKnown = prefs.isKey("panRaw0");   panRawAtHome  = prefs.getFloat("panRaw0", 0.0f);
    tiltRawAtHomeKnown = prefs.isKey("tiltRaw0"); tiltRawAtHome = prefs.getFloat("tiltRaw0", 0.0f);
    positionKnown = prefs.getBool("posOk", false);
    panPositionDeg = prefs.getFloat("panPos", 0.0f);
    tiltPositionDeg = prefs.getFloat("tiltPos", 0.0f);
    prefs.end();
    savedPositionKnown = lastSeenPositionKnown = positionKnown;
    savedPanDeg = lastSeenPanDeg = panPositionDeg;
    savedTiltDeg = lastSeenTiltDeg = tiltPositionDeg;
    positionSyncPending = panRawAtHomeKnown && tiltRawAtHomeKnown;

    setupWifi();
    setupBle();
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
    server.on("/status", handleStatus);
    server.on("/setWifiMode", handleSetWifiMode);
    server.on("/setWifiConfig", HTTP_POST, handleSetWifiConfig);
    server.begin();
}

void loop() {
    server.handleClient();
    updateButtonSerialMonitor();
    updatePhysicalButtons();
    maintainWifi();
    updateGoTo();
    refreshRawPosition();
    savePositionWhenSettled();
    updateBle();
    if (wifiRestartAtMs && (long)(millis() - wifiRestartAtMs) >= 0) ESP.restart();

    PelcoCommand pelcoCmd;
    if (pelco.poll(pelcoCmd)) {
        handlePelcoCommand(pelcoCmd);
    }

    updateDisplay();
}
