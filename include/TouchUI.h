#pragma once
// On-device touch UI for the 4" ST7796S/XPT2046 panel - a local equivalent
// of the web UI in web_page.h, split across two screens (MAIN: status +
// jog + Home/Azimut, SETTINGS: speed/presets/calibration/limits) since
// there isn't room for all of it on one 480x320 screen at once.
//
// This header only draws/reads the touch screen and dispatches to the same
// action functions the web UI's HTTP handlers use (doStep(), doHome(),
// etc., all defined in main.cpp) - it holds no pan/tilt state of its own,
// it just reads the same globals main.cpp already maintains (via the
// `extern` declarations below) so both UIs always agree.

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7796S.h>
#include <XPT2046_Touchscreen.h>
#include "config.h"

// ---------- State this UI reads, owned/updated by main.cpp ----------
extern bool compassOk;
extern String wifiLabel;
extern String ipStr;
extern bool homeAzimuthSet;
extern float panPositionDeg, tiltPositionDeg;
extern bool positionKnown;
extern float rawPanDeg, rawTiltDeg;
extern bool rawPanKnown, rawTiltKnown;
extern uint8_t jogSpeed;
extern CalPhase calPhase;
extern String calMessage;
extern bool panLimitMinKnown, panLimitMaxKnown, tiltLimitMinKnown, tiltLimitMaxKnown;
extern float panLimitMinDeg, panLimitMaxDeg, tiltLimitMinDeg, tiltLimitMaxDeg;
extern GoAxis goToAxis;
extern GoSource goSource;
extern float presetAz[PELCO_USER_PRESET_COUNT];
extern float presetTilt[PELCO_USER_PRESET_COUNT];
extern bool presetAzKnown[PELCO_USER_PRESET_COUNT];
extern bool presetTiltKnown[PELCO_USER_PRESET_COUNT];
extern bool azimuthAvailable();
extern float currentAzimuth();

// ---------- Actions this UI calls, defined in main.cpp ----------
extern bool doStep(bool pan, int dir);
extern void doHome();
extern void doSaveHome();
extern void doClearHome();
extern GoAzimuthResult doGoAzimuth(float targetDeg);
extern void doGoToPanMid();
extern void doGoToTiltZero();
extern void doPresetSet(uint8_t num);
extern void doPresetGo(uint8_t num);
extern void doSetSpeed(uint8_t v);
extern void startAutoCalibrate();
extern void doCancelCalibrate();
extern bool doCalLimit(bool pan, bool isMin);

struct Btn {
    int16_t x, y, w, h;
    const char *label;
    uint8_t id;
    uint8_t size;   // text size (1 or 2)
    uint16_t color; // idle fill color; some buttons override this dynamically
    bool visible;
};

enum BtnId {
    BTN_NONE = 0,
    BTN_SETTINGS_NAV,
    BTN_DPAD_UP, BTN_DPAD_DOWN, BTN_DPAD_LEFT, BTN_DPAD_RIGHT,
    BTN_AZ_MINUS10, BTN_AZ_MINUS1, BTN_AZ_PLUS1, BTN_AZ_PLUS10, BTN_AZ_GO,
    BTN_HOME, BTN_SAVE_HOME, BTN_CLEAR_HOME,
    BTN_BACK_NAV,
    BTN_SPEED_MINUS10, BTN_SPEED_MINUS1, BTN_SPEED_PLUS1, BTN_SPEED_PLUS10,
    BTN_PRESET_SAVE_0, BTN_PRESET_GO_0,
    BTN_PRESET_SAVE_1, BTN_PRESET_GO_1,
    BTN_PRESET_SAVE_2, BTN_PRESET_GO_2,
    BTN_AUTO_CAL, BTN_CANCEL_CAL,
    BTN_GOTO_PAN_MID, BTN_GOTO_TILT_ZERO,
    BTN_PAN_LIMIT_MIN, BTN_PAN_LIMIT_MAX,
    BTN_TILT_LIMIT_MIN, BTN_TILT_LIMIT_MAX,
    BTN_CONFIRM_YES, BTN_CONFIRM_NO,
};

enum ConfirmAction {
    CONF_NONE, CONF_SAVE_HOME, CONF_CLEAR_HOME, CONF_AUTO_CAL,
    CONF_PRESET_SAVE_0, CONF_PRESET_SAVE_1, CONF_PRESET_SAVE_2,
};

class TouchUI {
public:
    TouchUI() : tft(&SPI, TFT_CS_PIN, TFT_DC_PIN, TFT_RST_PIN), touch(TOUCH_CS_PIN, TOUCH_IRQ_PIN) {}

    void begin() {
        pinMode(TFT_BLK_PIN, OUTPUT);
        digitalWrite(TFT_BLK_PIN, HIGH); // backlight on

        // Explicit pins (not ESP32-S3 hw-SPI defaults - see TFT_MISO_PIN in
        // config.h) - must happen before tft.init()/touch.begin(), which
        // both just call SPI.begin() again with no args internally; that's
        // a safe no-op once the bus is already started with these pins.
        Serial.println("touchUI checkpoint: before SPI.begin");
        SPI.begin(TFT_SCK_PIN, TFT_MISO_PIN, TFT_MOSI_PIN, TFT_CS_PIN);
        Serial.println("touchUI checkpoint: before tft.init");

        // tft.init() always runs its one-time init command sequence at the
        // library's own hardcoded 32MHz default, regardless of any prior
        // setSPISpeed() call (init()->commonInit()->begin() unconditionally
        // resets it) - setSPISpeed() only takes effect for draw calls made
        // AFTER init(), so it must come after, not before.
        tft.init();           // native 320x480 portrait
        Serial.println("touchUI checkpoint: after tft.init");
        tft.setSPISpeed(TFT_SPI_SPEED_HZ);
        tft.setRotation(1);   // landscape, 480x320 - try 3 if the image is upside down
        Serial.println("touchUI checkpoint: before fillScreen");
        tft.fillScreen(ST77XX_BLACK);
        Serial.println("touchUI checkpoint: after fillScreen");

        touch.begin(); // this library version always uses the global SPI object, already reconfigured above
        Serial.println("touchUI checkpoint: after touch.begin");
        // Defensive: touch.begin() sets this pin plain INPUT with no pull-up.
        // If the touch module doesn't have its own pull-up on T_IRQ, the pin
        // floats and reads garbage; readTouch() no longer depends on it (see
        // the isrWake note there) but a pull-up still keeps it from floating.
        pinMode(TOUCH_IRQ_PIN, INPUT_PULLUP);

        colGreen = tft.color565(0x2b, 0x9e, 0x4f);
        colRed = tft.color565(0xc0, 0x39, 0x2b); // kept only for the "driving" motion indicator, see updateGoButton()

        initButtons();
        Serial.println("touchUI checkpoint: before drawScreen");
        drawScreen(true);
        Serial.println("touchUI checkpoint: after drawScreen");
    }

    void update() {
        int16_t sx, sy;
        bool down = readTouch(sx, sy);
        if (down && !touchWasDown) {
            Serial.printf("touch raw=(%d,%d) screen=(%d,%d)\n", lastRawX, lastRawY, sx, sy);
            handlePress(sx, sy);
        }
        touchWasDown = down;
        printTouchDiag();

        refreshDynamic(false);
    }

    // Temporary bring-up diagnostic: prints the raw z (pressure) reading
    // whether or not it crosses the touch threshold, so wiring problems are
    // visible even when nothing ever registers as a "press". Remove once
    // touch is confirmed working.
    //   - always near 0, never rises when pressing firmly with a stylus -> a
    //     wiring problem on T_CS/T_CLK/T_DIN/T_DO (SPI comms to the chip
    //     itself aren't happening) - double check those against TOUCH_CS_PIN/
    //     TFT_SCK_PIN/TFT_MOSI_PIN/TFT_MISO_PIN in config.h.
    //   - jumps around randomly even untouched -> a floating/disconnected
    //     pin, most likely T_DO (MISO, TFT_MISO_PIN=18).
    //   - stays at exactly 0 or exactly 4095 permanently -> MISO stuck
    //     high/low (shorted, or not connected and reading whatever the bus
    //     was last driven to).
    //   - near 0 idle, jumps above ~400 when pressed -> working correctly;
    //     the problem is TOUCH_RAW_MINX/MAXX/MINY/MAXY calibration instead.
    unsigned long lastDiagMs = 0;
    void printTouchDiag() {
        if (millis() - lastDiagMs < 500) return;
        lastDiagMs = millis();
        touch.isrWake = true;
        TS_Point p = touch.getPoint();
        Serial.printf("touch diag: z=%d x=%d y=%d touched=%d\n", p.z, p.x, p.y, touch.touched());
    }

private:
    Adafruit_ST7796S tft;
    XPT2046_Touchscreen touch;
    uint16_t colGreen, colRed; // colRed is only used for the "driving" motion indicator

    enum Screen { SCR_MAIN, SCR_SETTINGS } screen = SCR_MAIN;
    bool touchWasDown = false;
    unsigned long lastDynamicMs = 0;

    static const uint8_t MAIN_BTN_COUNT = 16;
    static const uint8_t SETTINGS_BTN_COUNT = 19;
    Btn mainButtons[MAIN_BTN_COUNT];
    Btn settingsButtons[SETTINGS_BTN_COUNT];

    float azimuthTarget = 0.0f;

    bool confirmActive = false;
    ConfirmAction pendingConfirm = CONF_NONE;
    const char *confirmLine1 = "";
    const char *confirmLine2 = "";
    Btn confirmButtons[2];

    // Forces every dynamic field to redraw next refreshDynamic() call -
    // used after anything that clears/repaints the screen (screen switch,
    // closing the confirm modal), since the "last shown" caches below would
    // otherwise think nothing changed and skip redrawing over blank pixels.
    bool forceDynamicRedraw = true;

    // ---------- "last shown" caches, so refreshDynamic() only repaints
    // fields that actually changed (avoids visible flicker on a shared bus
    // that's also carrying RS485/web traffic). NAN/impossible sentinels so
    // the first call after a screen switch always redraws.
    float lastHeadingShown = -9999, lastPanShown = -9999, lastRawPanShown = -9999, lastTiltRelShown = -9999;
    bool lastAzOkShown = true, lastPosKnownShown = true, lastRawPanOkShown = true, lastTiltRelOkShown = true;
    bool lastHomeSetShown = true;
    float lastAzTargetShown = -9999;
    uint8_t lastSpeedShown = 255;
    String lastCalMsgShown = "\x01"; // sentinel that can't match a real message
    bool lastCalActiveShown = true;
    bool lastPresetAzKnownShown[PELCO_USER_PRESET_COUNT] = {true, true, true};
    bool lastPresetTiltKnownShown[PELCO_USER_PRESET_COUNT] = {true, true, true};
    float lastPresetAzShown[PELCO_USER_PRESET_COUNT] = {-9999, -9999, -9999};
    float lastPresetTiltShown[PELCO_USER_PRESET_COUNT] = {-9999, -9999, -9999};
    bool lastPanLimitsShown = true, lastTiltLimitsShown = true; // forces first draw

    void initButtons() {
        // Status block occupies x=8-280, y=26-100 (see refreshMainDynamic) -
        // the D-pad sits beside it on the right, both above the preset/
        // azimuth/home rows stacked below.
        uint8_t i = 0;
        mainButtons[i++] = {376, 2, 100, 22, "SETTINGS", BTN_SETTINGS_NAV, 1, colGreen, true};
        mainButtons[i++] = {350, 26, 60, 38, "UP", BTN_DPAD_UP, 2, colGreen, true};
        mainButtons[i++] = {290, 68, 52, 46, "<", BTN_DPAD_LEFT, 2, colGreen, true};
        mainButtons[i++] = {404, 68, 52, 46, ">", BTN_DPAD_RIGHT, 2, colGreen, true};
        mainButtons[i++] = {350, 118, 60, 38, "DN", BTN_DPAD_DOWN, 2, colGreen, true};
        mainButtons[i++] = {8, 162, 149, 32, "P1", BTN_PRESET_GO_0, 2, colGreen, true};
        mainButtons[i++] = {165, 162, 149, 32, "P2", BTN_PRESET_GO_1, 2, colGreen, true};
        mainButtons[i++] = {322, 162, 150, 32, "P3", BTN_PRESET_GO_2, 2, colGreen, true};
        mainButtons[i++] = {8, 200, 44, 32, "-10", BTN_AZ_MINUS10, 1, colGreen, true};
        mainButtons[i++] = {56, 200, 44, 32, "-1", BTN_AZ_MINUS1, 1, colGreen, true};
        mainButtons[i++] = {208, 200, 44, 32, "+1", BTN_AZ_PLUS1, 1, colGreen, true};
        mainButtons[i++] = {256, 200, 44, 32, "+10", BTN_AZ_PLUS10, 1, colGreen, true};
        mainButtons[i++] = {304, 200, 168, 32, "GO AZ", BTN_AZ_GO, 2, colGreen, true};
        mainButtons[i++] = {8, 238, 150, 38, "HOME", BTN_HOME, 2, colGreen, true};
        mainButtons[i++] = {164, 238, 150, 38, "SAVE HOME", BTN_SAVE_HOME, 1, colGreen, true};
        mainButtons[i++] = {320, 238, 152, 38, "CLEAR HOME", BTN_CLEAR_HOME, 1, colGreen, true};

        i = 0;
        settingsButtons[i++] = {4, 2, 90, 26, "BACK", BTN_BACK_NAV, 1, colGreen, true};
        settingsButtons[i++] = {110, 32, 40, 26, "-10", BTN_SPEED_MINUS10, 1, colGreen, true};
        settingsButtons[i++] = {154, 32, 40, 26, "-1", BTN_SPEED_MINUS1, 1, colGreen, true};
        settingsButtons[i++] = {252, 32, 40, 26, "+1", BTN_SPEED_PLUS1, 1, colGreen, true};
        settingsButtons[i++] = {296, 32, 40, 26, "+10", BTN_SPEED_PLUS10, 1, colGreen, true};
        settingsButtons[i++] = {244, 66, 90, 28, "SAVE", BTN_PRESET_SAVE_0, 1, colGreen, true};
        settingsButtons[i++] = {340, 66, 90, 28, "GO", BTN_PRESET_GO_0, 1, colGreen, true};
        settingsButtons[i++] = {244, 100, 90, 28, "SAVE", BTN_PRESET_SAVE_1, 1, colGreen, true};
        settingsButtons[i++] = {340, 100, 90, 28, "GO", BTN_PRESET_GO_1, 1, colGreen, true};
        settingsButtons[i++] = {244, 134, 90, 28, "SAVE", BTN_PRESET_SAVE_2, 1, colGreen, true};
        settingsButtons[i++] = {340, 134, 90, 28, "GO", BTN_PRESET_GO_2, 1, colGreen, true};
        settingsButtons[i++] = {8, 170, 150, 30, "AUTO CAL", BTN_AUTO_CAL, 1, colGreen, true};
        settingsButtons[i++] = {8, 170, 150, 30, "CANCEL", BTN_CANCEL_CAL, 1, colGreen, false};
        settingsButtons[i++] = {8, 204, 230, 30, "PAN MID", BTN_GOTO_PAN_MID, 1, colGreen, true};
        settingsButtons[i++] = {244, 204, 230, 30, "TILT ZERO", BTN_GOTO_TILT_ZERO, 1, colGreen, true};
        settingsButtons[i++] = {244, 238, 110, 30, "SET MIN", BTN_PAN_LIMIT_MIN, 1, colGreen, true};
        settingsButtons[i++] = {360, 238, 110, 30, "SET MAX", BTN_PAN_LIMIT_MAX, 1, colGreen, true};
        settingsButtons[i++] = {244, 272, 110, 30, "SET MIN", BTN_TILT_LIMIT_MIN, 1, colGreen, true};
        settingsButtons[i++] = {360, 272, 110, 30, "SET MAX", BTN_TILT_LIMIT_MAX, 1, colGreen, true};

        confirmButtons[0] = {110, 190, 110, 32, "YES", BTN_CONFIRM_YES, 2, colGreen, true};
        confirmButtons[1] = {260, 190, 110, 32, "NO", BTN_CONFIRM_NO, 2, colGreen, true};
    }

    // ---------- touch input ----------
    // Raw XPT2046 ADC coordinates (~0-4095), not screen pixels - mapped via
    // the calibration constants in config.h. If touches land offset or
    // mirrored, the "touch press" line this prints tells you the mapped
    // point actually used, to help re-tune TOUCH_RAW_MIN/MAXX/Y or
    // TOUCH_SWAP_XY.
    int16_t lastRawX = 0, lastRawY = 0; // last raw ADC reading, kept only for the calibration Serial print in update()

    bool readTouch(int16_t &sx, int16_t &sy) {
        // The installed XPT2046_Touchscreen version only actually polls the
        // chip over SPI when its internal isrWake latch is true - normally
        // that latch is set by a hardware interrupt on T_IRQ, and once it
        // goes false (e.g. the very first check at boot, screen untouched)
        // it stays false forever unless that interrupt fires. If T_IRQ is
        // unwired, floating, or just unreliable, touched() then silently
        // never works again. isrWake is technically public in this version
        // (the "// protected:" above it in the header is commented out, so
        // it never took effect) - forcing it true before every check turns
        // this into plain SPI polling and makes touch work regardless of
        // T_IRQ wiring.
        touch.isrWake = true;
        if (!touch.touched()) return false;
        TS_Point p = touch.getPoint();
        int16_t rx = p.x, ry = p.y;
#if TOUCH_SWAP_XY
        int16_t t = rx; rx = ry; ry = t;
#endif
        lastRawX = rx; lastRawY = ry;
        long mx = map(rx, TOUCH_RAW_MINX, TOUCH_RAW_MAXX, 0, tft.width());
        long my = map(ry, TOUCH_RAW_MINY, TOUCH_RAW_MAXY, 0, tft.height());
        sx = constrain((int)mx, 0, tft.width() - 1);
        sy = constrain((int)my, 0, tft.height() - 1);
        return true;
    }

    static bool hit(const Btn &b, int16_t x, int16_t y) {
        return b.visible && x >= b.x && x < b.x + b.w && y >= b.y && y < b.y + b.h;
    }

    uint8_t hitTest(Btn *btns, uint8_t count, int16_t x, int16_t y) {
        for (uint8_t i = 0; i < count; i++) {
            if (hit(btns[i], x, y)) return btns[i].id;
        }
        return BTN_NONE;
    }

    // Looked up by id rather than a fixed array index so inserting/
    // reordering buttons in initButtons() can't silently point this at the
    // wrong button.
    static Btn &findBtn(Btn *btns, uint8_t count, uint8_t id) {
        for (uint8_t i = 0; i < count; i++) {
            if (btns[i].id == id) return btns[i];
        }
        return btns[0]; // unreachable as long as every id passed in exists in the array
    }

    void handlePress(int16_t x, int16_t y) {
        if (confirmActive) {
            uint8_t id = hitTest(confirmButtons, 2, x, y);
            if (id == BTN_CONFIRM_YES) {
                executeConfirmed();
                confirmActive = false;
                drawScreen(true);
            } else if (id == BTN_CONFIRM_NO) {
                confirmActive = false;
                drawScreen(true);
            }
            return;
        }
        uint8_t id = (screen == SCR_MAIN) ? hitTest(mainButtons, MAIN_BTN_COUNT, x, y)
                                           : hitTest(settingsButtons, SETTINGS_BTN_COUNT, x, y);
        if (screen == SCR_MAIN) handleMainPress(id); else handleSettingsPress(id);
    }

    static float wrap360(float v) {
        v = fmodf(v, 360.0f);
        if (v < 0) v += 360.0f;
        return v;
    }

    void setSpeedClamped(int v) {
        if (v < 1) v = 1;
        if (v > 63) v = 63;
        doSetSpeed((uint8_t)v);
    }

    void startConfirm(ConfirmAction action, const char *line1, const char *line2) {
        pendingConfirm = action;
        confirmLine1 = line1;
        confirmLine2 = line2;
        confirmActive = true;
        drawConfirmModal();
    }

    void executeConfirmed() {
        switch (pendingConfirm) {
            case CONF_SAVE_HOME:     doSaveHome();  break;
            case CONF_CLEAR_HOME:    doClearHome(); break;
            case CONF_AUTO_CAL:      startAutoCalibrate(); break;
            case CONF_PRESET_SAVE_0: doPresetSet(PELCO_USER_PRESET_BASE + 0); break;
            case CONF_PRESET_SAVE_1: doPresetSet(PELCO_USER_PRESET_BASE + 1); break;
            case CONF_PRESET_SAVE_2: doPresetSet(PELCO_USER_PRESET_BASE + 2); break;
            default: break;
        }
        pendingConfirm = CONF_NONE;
    }

    void handleMainPress(uint8_t id) {
        switch (id) {
            case BTN_SETTINGS_NAV: screen = SCR_SETTINGS; drawScreen(true); break;
            case BTN_DPAD_UP:    doStep(false, 1);  break;
            case BTN_DPAD_DOWN:  doStep(false, -1); break;
            case BTN_DPAD_LEFT:  doStep(true, -1);  break;
            case BTN_DPAD_RIGHT: doStep(true, 1);   break;
            case BTN_AZ_MINUS10: azimuthTarget = wrap360(azimuthTarget - 10); break;
            case BTN_AZ_MINUS1:  azimuthTarget = wrap360(azimuthTarget - 1);  break;
            case BTN_AZ_PLUS1:   azimuthTarget = wrap360(azimuthTarget + 1);  break;
            case BTN_AZ_PLUS10:  azimuthTarget = wrap360(azimuthTarget + 10); break;
            case BTN_AZ_GO: doGoAzimuth(azimuthTarget); break;
            case BTN_HOME: doHome(); break;
            case BTN_SAVE_HOME:
                startConfirm(CONF_SAVE_HOME, "Save current position", "and heading as Home?");
                break;
            case BTN_CLEAR_HOME:
                startConfirm(CONF_CLEAR_HOME, "Clear saved Home?", "Azimut reverts to compass.");
                break;
            // Quick recall only - saving a preset stays on the Settings
            // screen (it needs a confirm, see handleSettingsPress()).
            case BTN_PRESET_GO_0: doPresetGo(PELCO_USER_PRESET_BASE + 0); break;
            case BTN_PRESET_GO_1: doPresetGo(PELCO_USER_PRESET_BASE + 1); break;
            case BTN_PRESET_GO_2: doPresetGo(PELCO_USER_PRESET_BASE + 2); break;
            default: break;
        }
    }

    void handleSettingsPress(uint8_t id) {
        switch (id) {
            case BTN_BACK_NAV: screen = SCR_MAIN; drawScreen(true); break;
            case BTN_SPEED_MINUS10: setSpeedClamped(jogSpeed - 10); break;
            case BTN_SPEED_MINUS1:  setSpeedClamped(jogSpeed - 1);  break;
            case BTN_SPEED_PLUS1:   setSpeedClamped(jogSpeed + 1);  break;
            case BTN_SPEED_PLUS10:  setSpeedClamped(jogSpeed + 10); break;
            case BTN_PRESET_SAVE_0: startConfirm(CONF_PRESET_SAVE_0, "Save current position", "as Preset 1?"); break;
            case BTN_PRESET_SAVE_1: startConfirm(CONF_PRESET_SAVE_1, "Save current position", "as Preset 2?"); break;
            case BTN_PRESET_SAVE_2: startConfirm(CONF_PRESET_SAVE_2, "Save current position", "as Preset 3?"); break;
            case BTN_PRESET_GO_0: doPresetGo(PELCO_USER_PRESET_BASE + 0); break;
            case BTN_PRESET_GO_1: doPresetGo(PELCO_USER_PRESET_BASE + 1); break;
            case BTN_PRESET_GO_2: doPresetGo(PELCO_USER_PRESET_BASE + 2); break;
            case BTN_AUTO_CAL:
                startConfirm(CONF_AUTO_CAL, "Auto Calibrate will drive", "pan/tilt to their limits.");
                break;
            case BTN_CANCEL_CAL: doCancelCalibrate(); break;
            case BTN_GOTO_PAN_MID: doGoToPanMid(); break;
            case BTN_GOTO_TILT_ZERO: doGoToTiltZero(); break;
            case BTN_PAN_LIMIT_MIN:  doCalLimit(true, true);   break;
            case BTN_PAN_LIMIT_MAX:  doCalLimit(true, false);  break;
            case BTN_TILT_LIMIT_MIN: doCalLimit(false, true);  break;
            case BTN_TILT_LIMIT_MAX: doCalLimit(false, false); break;
            default: break;
        }
    }

    // ---------- drawing ----------
    void drawButton(const Btn &b, uint16_t bg) {
        if (!b.visible) return;
        tft.fillRoundRect(b.x, b.y, b.w, b.h, 4, bg);
        tft.setTextSize(b.size);
        tft.setTextColor(ST77XX_WHITE, bg);
        int16_t x1, y1; uint16_t tw, th;
        tft.getTextBounds(b.label, 0, 0, &x1, &y1, &tw, &th);
        tft.setCursor(b.x + (b.w - (int16_t)tw) / 2 - x1, b.y + (b.h - (int16_t)th) / 2 - y1);
        tft.print(b.label);
    }

    void drawField(int16_t x, int16_t y, int16_t w, int16_t h, const char *text, uint16_t fg, uint8_t size = 1) {
        tft.fillRect(x, y, w, h, ST77XX_BLACK);
        tft.setTextSize(size);
        tft.setTextColor(fg, ST77XX_BLACK);
        tft.setCursor(x, y);
        tft.print(text);
    }

    // GO-style button: red while actively auto-driving toward its target
    // (goToAxis/goSource, set by doGoAzimuth()/doGoToPanMid()/doGoToTiltZero()
    // in main.cpp), green otherwise.
    void updateGoButton(Btn &b, bool nowDriving) {
        drawButton(b, nowDriving ? colRed : colGreen);
    }

    void drawScreen(bool full) {
        if (full) {
            tft.fillScreen(ST77XX_BLACK);
            if (screen == SCR_MAIN) drawMainStatic(); else drawSettingsStatic();
            forceDynamicRedraw = true;
        }
        refreshDynamic(true);
    }

    void drawMainStatic() {
        char buf[48];
        snprintf(buf, sizeof(buf), "%s  %s", wifiLabel.c_str(), ipStr.c_str());
        tft.setTextSize(1);
        tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
        tft.setCursor(4, 8);
        tft.print(buf);
        for (uint8_t i = 0; i < MAIN_BTN_COUNT; i++) {
            const Btn &b = mainButtons[i];
            if (b.id == BTN_AZ_GO) continue; // dynamic color, drawn by refreshDynamic()
            drawButton(b, b.color);
        }
    }

    void drawSettingsStatic() {
        tft.setTextSize(1);
        tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
        tft.setCursor(150, 8);
        tft.print("SETTINGS");
        tft.setCursor(8, 40);
        tft.print("SPEED");
        for (uint8_t i = 0; i < SETTINGS_BTN_COUNT; i++) {
            const Btn &b = settingsButtons[i];
            if (b.id == BTN_GOTO_PAN_MID || b.id == BTN_GOTO_TILT_ZERO) continue; // dynamic color
            if (b.id == BTN_AUTO_CAL || b.id == BTN_CANCEL_CAL) continue;         // dynamic visibility
            drawButton(b, b.color);
        }
    }

    void drawConfirmModal() {
        tft.fillRect(90, 90, 300, 140, tft.color565(0x22, 0x22, 0x22));
        tft.drawRect(90, 90, 300, 140, ST77XX_WHITE);
        tft.setTextSize(1);
        tft.setTextColor(ST77XX_WHITE, tft.color565(0x22, 0x22, 0x22));
        tft.setCursor(104, 110);
        tft.print(confirmLine1);
        tft.setCursor(104, 128);
        tft.print(confirmLine2);
        drawButton(confirmButtons[0], confirmButtons[0].color);
        drawButton(confirmButtons[1], confirmButtons[1].color);
    }

    static void fmtSigned(char *buf, size_t n, float v) {
        snprintf(buf, n, "%s%.1f", v >= 0 ? "+" : "", v);
    }

    static void fmtLimits(char *buf, size_t n, const char *axis, bool minOk, float minV, bool maxOk, float maxV) {
        char a[10], b[10];
        snprintf(a, sizeof(a), minOk ? "%.1f" : "--", minV);
        snprintf(b, sizeof(b), maxOk ? "%.1f" : "--", maxV);
        snprintf(buf, n, "%s min %s max %s", axis, a, b);
    }

    static void fmtPreset(char *buf, size_t n, uint8_t idx) {
        char az[12], tl[12];
        if (presetAzKnown[idx]) snprintf(az, sizeof(az), "%.1f", presetAz[idx]); else snprintf(az, sizeof(az), "--");
        if (presetTiltKnown[idx]) { char s[10]; fmtSigned(s, sizeof(s), presetTilt[idx]); snprintf(tl, sizeof(tl), "%s", s); }
        else snprintf(tl, sizeof(tl), "--");
        snprintf(buf, n, "P%d Az %s Tilt %s", idx + 1, az, tl);
    }

    // Redraws only fields whose value changed since last time (rate-limited
    // unless force=true, e.g. right after a full screen redraw) - keeps the
    // shared SPI bus free for RS485/web traffic and avoids visible flicker.
    void refreshDynamic(bool force) {
        if (confirmActive) return; // don't paint over the modal
        if (!force) {
            if (millis() - lastDynamicMs < 200) return;
            lastDynamicMs = millis();
        }
        bool f = force || forceDynamicRedraw;
        forceDynamicRedraw = false;

        if (screen == SCR_MAIN) refreshMainDynamic(f); else refreshSettingsDynamic(f);
    }

    void refreshMainDynamic(bool f) {
        char buf[32];
        bool azOk = azimuthAvailable();
        float heading = azOk ? currentAzimuth() : lastHeadingShown;

        if (f || azOk != lastAzOkShown || fabs(heading - lastHeadingShown) >= 0.05f || homeAzimuthSet != lastHomeSetShown) {
            lastAzOkShown = azOk; lastHeadingShown = heading; lastHomeSetShown = homeAzimuthSet;
            if (azOk) {
                snprintf(buf, sizeof(buf), "Azimut %.1f%s", heading, homeAzimuthSet ? "" : " (no home)");
                drawField(8, 26, 268, 18, buf, ST77XX_WHITE, 2);
            } else {
                drawField(8, 26, 268, 18, "NO COMPASS", ST77XX_WHITE, 2);
            }
        }

        if (f || positionKnown != lastPosKnownShown || fabs(panPositionDeg - lastPanShown) >= 0.05f) {
            lastPosKnownShown = positionKnown; lastPanShown = panPositionDeg;
            if (positionKnown) snprintf(buf, sizeof(buf), "Pan  %.1f", panPositionDeg);
            else snprintf(buf, sizeof(buf), "Pan  ??");
            drawField(8, 44, 268, 18, buf, ST77XX_WHITE, 2);
        }

        if (f || rawPanKnown != lastRawPanOkShown || fabs(rawPanDeg - lastRawPanShown) >= 0.05f) {
            lastRawPanOkShown = rawPanKnown; lastRawPanShown = rawPanDeg;
            if (rawPanKnown) snprintf(buf, sizeof(buf), "Motor Pan %.1f", rawPanDeg);
            else snprintf(buf, sizeof(buf), "Motor Pan ??");
            drawField(8, 62, 268, 18, buf, ST77XX_WHITE, 2);
        }

        float tiltRel = rawTiltDeg - TILT_ZERO_TARGET_DEG;
        if (f || rawTiltKnown != lastTiltRelOkShown || fabs(tiltRel - lastTiltRelShown) >= 0.05f) {
            lastTiltRelOkShown = rawTiltKnown; lastTiltRelShown = tiltRel;
            if (rawTiltKnown) { char s[10]; fmtSigned(s, sizeof(s), tiltRel); snprintf(buf, sizeof(buf), "Tilt %s", s); }
            else snprintf(buf, sizeof(buf), "Tilt ??");
            drawField(8, 80, 268, 18, buf, ST77XX_WHITE, 2);
        }

        if (f || fabs(azimuthTarget - lastAzTargetShown) >= 0.05f) {
            lastAzTargetShown = azimuthTarget;
            snprintf(buf, sizeof(buf), "%.0f", azimuthTarget);
            drawField(104, 200, 100, 26, buf, ST77XX_WHITE, 2);
        }

        bool azDriving = (goToAxis == GO_PAN && goSource == GO_SRC_AZIMUTH);
        updateGoButton(findBtn(mainButtons, MAIN_BTN_COUNT, BTN_AZ_GO), azDriving);
    }

    void refreshSettingsDynamic(bool f) {
        char buf[48];

        if (f || jogSpeed != lastSpeedShown) {
            lastSpeedShown = jogSpeed;
            snprintf(buf, sizeof(buf), "%u", jogSpeed);
            drawField(198, 32, 50, 26, buf, ST77XX_WHITE, 2);
        }

        for (uint8_t i = 0; i < PELCO_USER_PRESET_COUNT; i++) {
            if (f || presetAzKnown[i] != lastPresetAzKnownShown[i] || presetTiltKnown[i] != lastPresetTiltKnownShown[i] ||
                fabs(presetAz[i] - lastPresetAzShown[i]) >= 0.05f || fabs(presetTilt[i] - lastPresetTiltShown[i]) >= 0.05f) {
                lastPresetAzKnownShown[i] = presetAzKnown[i];
                lastPresetTiltKnownShown[i] = presetTiltKnown[i];
                lastPresetAzShown[i] = presetAz[i];
                lastPresetTiltShown[i] = presetTilt[i];
                fmtPreset(buf, sizeof(buf), i);
                drawField(8, 74 + i * 34, 230, 18, buf, ST77XX_WHITE, 1);
            }
        }

        bool calActive = (calPhase != CAL_IDLE);
        if (f || calActive != lastCalActiveShown) {
            lastCalActiveShown = calActive;
            Btn &autoCalBtn = findBtn(settingsButtons, SETTINGS_BTN_COUNT, BTN_AUTO_CAL);
            Btn &cancelCalBtn = findBtn(settingsButtons, SETTINGS_BTN_COUNT, BTN_CANCEL_CAL);
            autoCalBtn.visible = !calActive;
            cancelCalBtn.visible = calActive;
            tft.fillRect(8, 170, 150, 30, ST77XX_BLACK); // clear whichever one isn't drawn
            drawButton(calActive ? cancelCalBtn : autoCalBtn, calActive ? cancelCalBtn.color : autoCalBtn.color);
        }
        if (f || calMessage != lastCalMsgShown) {
            lastCalMsgShown = calMessage;
            drawField(166, 178, 306, 16, calMessage.c_str(), ST77XX_WHITE, 1);
        }

        bool panMidDriving = (goToAxis == GO_PAN && goSource == GO_SRC_PAN_MID);
        updateGoButton(findBtn(settingsButtons, SETTINGS_BTN_COUNT, BTN_GOTO_PAN_MID), panMidDriving);
        bool tiltZeroDriving = (goToAxis == GO_TILT && goSource == GO_SRC_TILT_ZERO);
        updateGoButton(findBtn(settingsButtons, SETTINGS_BTN_COUNT, BTN_GOTO_TILT_ZERO), tiltZeroDriving);

        bool panLimitsNow = panLimitMinKnown || panLimitMaxKnown;
        if (f || panLimitsNow != lastPanLimitsShown || panLimitMinDeg != lastPanLimitMinShown || panLimitMaxDeg != lastPanLimitMaxShown) {
            lastPanLimitsShown = panLimitsNow;
            lastPanLimitMinShown = panLimitMinDeg;
            lastPanLimitMaxShown = panLimitMaxDeg;
            fmtLimits(buf, sizeof(buf), "Pan", panLimitMinKnown, panLimitMinDeg, panLimitMaxKnown, panLimitMaxDeg);
            drawField(8, 246, 230, 18, buf, ST77XX_WHITE, 1);
        }
        bool tiltLimitsNow = tiltLimitMinKnown || tiltLimitMaxKnown;
        if (f || tiltLimitsNow != lastTiltLimitsShown || tiltLimitMinDeg != lastTiltLimitMinShown || tiltLimitMaxDeg != lastTiltLimitMaxShown) {
            lastTiltLimitsShown = tiltLimitsNow;
            lastTiltLimitMinShown = tiltLimitMinDeg;
            lastTiltLimitMaxShown = tiltLimitMaxDeg;
            fmtLimits(buf, sizeof(buf), "Tilt", tiltLimitMinKnown, tiltLimitMinDeg, tiltLimitMaxKnown, tiltLimitMaxDeg);
            drawField(8, 280, 230, 18, buf, ST77XX_WHITE, 1);
        }
    }

    float lastPanLimitMinShown = -99999, lastPanLimitMaxShown = -99999;
    float lastTiltLimitMinShown = -99999, lastTiltLimitMaxShown = -99999;
};
