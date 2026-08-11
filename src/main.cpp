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
//   TFT CS            5      ST7735, 128x160
//   TFT DC            17
//   TFT RST           16
//   TFT SCL           12     module's SPI clock pin; hw SPI (ESP32-S3 default pin)
//   TFT SDA           11     module's SPI data pin; hw SPI (ESP32-S3 default pin)
//   TFT BLK           6      backlight, driven HIGH = on
//   Compass SDA       21     QMC5883P, I2C addr 0x2C
//   Compass SCL       14
//   RS485 RX (RO)     4      UART2, Pelco-D - from the pan-tilt unit
//   RS485 TX (DI)     13     UART2, Pelco-D - to the pan-tilt unit
//   RS485 DE+/RE      15     module's DE and /RE pins tied together here

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

bool compassOk = false;
bool wifiUsingFallbackAP = false;
String ipStr;
String wifiLabel; // SSID connected to, or "AP mode" if using the fallback
unsigned long lastCompassLogMs = 0;

bool homeAzimuthSet = false;
float homeAzimuth = 0.0f;

// Pan/tilt position, tracked in exact PELCO_STEP_DEG increments (see
// handleStep() - each press moves by exactly one step, timed rather than
// estimated from an arbitrary hold duration). 0/0 means "at home"; becomes
// stale/unknown after recalling a non-home preset, since we don't know that
// preset's position relative to home.
float panPositionDeg = 0.0f, tiltPositionDeg = 0.0f;
bool positionKnown = false; // true once Home/Save Home establishes a 0/0 reference

// Both axes are back at the saved home position - resets the position
// reference to 0/0.
void resetPositionToHome() {
    panPositionDeg = 0.0f;
    tiltPositionDeg = 0.0f;
    positionKnown = true;
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

// "Go to Azimut": drives pan automatically toward targetAzimuth, one
// PELCO_STEP_DEG pulse at a time, until within tolerance. Non-blocking
// (unlike handleStep()) so /status polling can report progress live and
// the web UI can flip the Go button from red to green on arrival - a
// single request that blocked until arrival could mean a very long wait
// for a large turn, with no feedback in the meantime.
bool autoDriveActive = false;
float targetAzimuth = 0.0f;
bool autoDrivePulsing = false;
unsigned long autoDrivePulseUntilMs = 0;

void stopAutoDrive() {
    if (autoDrivePulsing) pelco.sendStop(RS485_ADDRESS);
    autoDriveActive = false;
    autoDrivePulsing = false;
}

void updateAutoDrive() {
    if (!autoDriveActive) return;

    if (autoDrivePulsing) {
        if ((long)(millis() - autoDrivePulseUntilMs) < 0) return; // pulse still in flight
        pelco.sendStop(RS485_ADDRESS);
        autoDrivePulsing = false;
    }

    // Shortest signed distance to the target, in (-180, 180].
    float diff = fmodf(targetAzimuth - currentAzimuth() + 540.0f, 360.0f) - 180.0f;
    if (fabs(diff) < AZIMUTH_ARRIVE_TOLERANCE_DEG) {
        autoDriveActive = false; // arrived
        return;
    }

    int8_t dir = diff > 0 ? 1 : -1; // + increases Azimut (pan right), matches handleStep()
    pelco.sendMove(RS485_ADDRESS, dir < 0, dir > 0, false, false, PELCO_STEP_SPEED, PELCO_STEP_SPEED);
    panPositionDeg += dir * PELCO_STEP_DEG;
    autoDrivePulseUntilMs = millis() + PAN_STEP_MS;
    autoDrivePulsing = true;
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
// move command at full speed, block for the duration that takes at the
// unit's rated speed, then stop. Blocking is intentional and simplest here -
// the physical head genuinely needs that long to complete the step, and a
// single button press has nothing else useful to do meanwhile.
void handleStep() {
    stopAutoDrive(); // manual jog overrides any in-progress "Go to Azimut"

    String axis = server.arg("axis");
    int dir = server.arg("dir").toInt() >= 0 ? 1 : -1;

    if (axis == "pan") {
        pelco.sendMove(RS485_ADDRESS, dir < 0, dir > 0, false, false, PELCO_STEP_SPEED, PELCO_STEP_SPEED);
        delay(PAN_STEP_MS);
        pelco.sendStop(RS485_ADDRESS);
        panPositionDeg += dir * PELCO_STEP_DEG;
    } else if (axis == "tilt") {
        pelco.sendMove(RS485_ADDRESS, false, false, dir > 0, dir < 0, PELCO_STEP_SPEED, PELCO_STEP_SPEED);
        delay(TILT_STEP_MS);
        pelco.sendStop(RS485_ADDRESS);
        tiltPositionDeg += dir * PELCO_STEP_DEG;
    } else {
        server.send(400, "text/plain", "bad axis");
        return;
    }
    server.send(200, "text/plain", "ok");
}

// Safety/manual use (e.g. testing via curl) - the head already stops itself
// at the end of each step in handleStep(), so the web UI doesn't call this
// directly, though it also cancels any in-progress "Go to Azimut".
void handleStop() {
    stopAutoDrive();
    pelco.sendStop(RS485_ADDRESS);
    server.send(200, "text/plain", "ok");
}

// Drives pan automatically toward a target Azimut - see updateAutoDrive()
// in loop(). Requires home to already be set, since "Azimut" before that
// is just the live (and camera-irrelevant) compass reading.
void handleGoAzimuth() {
    if (!homeAzimuthSet) { server.send(409, "text/plain", "home not set"); return; }
    float t = fmodf(server.arg("target").toFloat(), 360.0f);
    if (t < 0) t += 360.0f;
    targetAzimuth = t;
    autoDriveActive = true;
    autoDrivePulsing = false;
    server.send(200, "text/plain", "ok");
}

void handleHome() {
    stopAutoDrive();
    pelco.callPreset(RS485_ADDRESS, PELCO_HOME_PRESET);
    resetPositionToHome(); // best-effort - assumes the head lands exactly at home
    server.send(200, "text/plain", "ok");
}

// Point the head where "home" should be (using the jog buttons), then call
// this: it tells the pan-tilt unit to remember the current position as its
// HOME preset, and saves the current compass heading as the home azimuth
// reference (persisted to flash) - the Azimut display then reads relative
// to this direction (0 = facing home) instead of raw compass heading.
void handleSaveHome() {
    stopAutoDrive();
    pelco.setPreset(RS485_ADDRESS, PELCO_HOME_PRESET);
    resetPositionToHome();

    int16_t cx, cy, cz;
    if (compassOk && compass.read(cx, cy, cz)) {
        homeAzimuth = compass.headingFromRaw(cx, cy);
        homeAzimuthSet = true;
        prefs.begin("pantilt", false);
        prefs.putFloat("homeAz", homeAzimuth);
        prefs.end();
    }
    server.send(200, "text/plain", "ok");
}

// Reverts to live compass tracking - undoes Save Home (including the
// persisted flash value), so Azimut goes back to following the compass
// live instead of being frozen/dead-reckoned.
void handleClearHome() {
    stopAutoDrive();
    homeAzimuthSet = false;
    homeAzimuth = 0.0f;
    prefs.begin("pantilt", false);
    prefs.remove("homeAz");
    prefs.end();

    positionKnown = false;
    panPositionDeg = 0.0f;
    tiltPositionDeg = 0.0f;
    server.send(200, "text/plain", "ok");
}

bool parsePresetNum(uint8_t &num) {
    int n = server.arg("num").toInt();
    if (n < 0 || n > 255) return false;
    num = (uint8_t)n;
    return true;
}

void handlePresetSet() {
    uint8_t num;
    if (!parsePresetNum(num)) { server.send(400, "text/plain", "bad preset"); return; }
    pelco.setPreset(RS485_ADDRESS, num);
    server.send(200, "text/plain", "ok");
}

void handlePresetGo() {
    uint8_t num;
    if (!parsePresetNum(num)) { server.send(400, "text/plain", "bad preset"); return; }
    stopAutoDrive();
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

void handleStatus() {
    bool azOk = azimuthAvailable();
    float heading = azOk ? currentAzimuth() : 0.0f;

    char buf[180];
    snprintf(buf, sizeof(buf),
        "{\"heading\":%.1f,\"compassOk\":%s,\"homeSet\":%s,"
        "\"pan\":%.1f,\"tilt\":%.1f,\"posKnown\":%s,\"autoDrive\":%s}",
        heading, azOk ? "true" : "false", homeAzimuthSet ? "true" : "false",
        panPositionDeg, tiltPositionDeg, positionKnown ? "true" : "false",
        autoDriveActive ? "true" : "false");
    server.send(200, "application/json", buf);
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

    if (millis() - lastUpdateMs < 300) return; // cap TFT refresh rate
    lastUpdateMs = millis();

    bool azOk = azimuthAvailable();
    float heading = azOk ? currentAzimuth() : lastHeading;
    float pan = panPositionDeg;
    float tilt = tiltPositionDeg;

    if (fabs(heading - lastHeading) < 0.5f && fabs(pan - lastPan) < 0.1f && fabs(tilt - lastTilt) < 0.1f) {
        return; // nothing meaningfully changed, skip redraw to avoid flicker
    }
    lastHeading = heading;
    lastPan = pan;
    lastTilt = tilt;

    tft.fillRect(0, 32, tft.width(), tft.height() - 32, ST77XX_BLACK);
    tft.setTextSize(2);
    tft.setTextColor(azOk ? ST77XX_CYAN : ST77XX_RED);
    tft.setCursor(4, 38);
    if (azOk) {
        tft.printf("Azimut %6.1f", heading);
    } else {
        tft.print("NO COMPASS");
    }

    tft.setTextColor(positionKnown ? ST77XX_YELLOW : ST77XX_RED);
    tft.setCursor(4, 58);
    if (positionKnown) {
        tft.printf("Pan  %6.1f", pan);
    } else {
        tft.print("Pan     ??");
    }
    tft.setCursor(4, 78);
    if (positionKnown) {
        tft.printf("Tilt %6.1f", tilt);
    } else {
        tft.print("Tilt    ??");
    }
}

void logCompass() {
    if (millis() - lastCompassLogMs < 1000) return;
    lastCompassLogMs = millis();
    if (azimuthAvailable()) {
        Serial.printf("Azimut=%.1f\n", currentAzimuth());
    } else {
        Serial.println("Compass read failed");
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

    pinMode(TFT_BLK_PIN, OUTPUT);
    digitalWrite(TFT_BLK_PIN, HIGH); // backlight on

    // Lowered from the 16MHz default - random colorful static that never
    // resolves into an image is the classic symptom of SPI signal integrity
    // problems on breadboard jumper wires; a slower clock is more tolerant
    // of that. Raise this back up once wiring is confirmed solid.
    tft.setSPISpeed(4000000);
    tft.initR(INITR_BLACKTAB); // switch to INITR_GREENTAB if colors look wrong
    tft.setRotation(1);        // landscape, 160x128

    compassOk = compass.begin();
    if (!compassOk) {
        Serial.println("QMC5883P not found - check wiring (SDA=21/SCL=14) and I2C address 0x2C");
        scanI2C();
    }

    pelco.begin(Serial2, RS485_RX_PIN, RS485_TX_PIN, RS485_DE_RE_PIN, RS485_BAUD);

    prefs.begin("pantilt", true);
    homeAzimuthSet = prefs.isKey("homeAz");
    homeAzimuth = prefs.getFloat("homeAz", 0.0f);
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
    server.on("/presetSet", handlePresetSet);
    server.on("/presetGo", handlePresetGo);
    server.on("/status", handleStatus);
    server.begin();
}

void loop() {
    server.handleClient();
    maintainWifi();
    updateAutoDrive();

    PelcoCommand pelcoCmd;
    if (pelco.poll(pelcoCmd)) {
        handlePelcoCommand(pelcoCmd);
    }

    updateDisplay();
    logCompass();
}
