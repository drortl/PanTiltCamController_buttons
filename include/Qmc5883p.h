#pragma once
// Register-level driver for the QMC5883P 3-axis digital compass (I2C addr 0x2C).
// Confirmed against a working reference sketch tested on this exact module -
// do not switch this back to genuine-HMC5883L registers/address (0x1E)
// without re-verifying on hardware; the board is labeled "HMC5883L" but
// that's reused stock product photography, not the actual chip.
// Unlike the HMC5883L, this chip returns axes in plain X, Y, Z order.

#include <Wire.h>
#include <math.h>
#include "config.h"

class Qmc5883p {
public:
    bool begin() {
        Wire.begin(COMPASS_SDA_PIN, COMPASS_SCL_PIN);
        writeReg(MODE_REG, 0xCF);   // continuous mode, 200Hz ODR, 8x oversampling
        writeReg(CONFIG_REG, 0x08); // set/reset on, 8G range
        Wire.beginTransmission(QMC5883P_ADDR);
        return Wire.endTransmission() == 0;
    }

    bool read(int16_t &x, int16_t &y, int16_t &z) {
        Wire.beginTransmission(QMC5883P_ADDR);
        Wire.write(X_LSB_REG);
        if (Wire.endTransmission(false) != 0) return false;
        if (Wire.requestFrom(QMC5883P_ADDR, 6) != 6) return false;
        x = Wire.read() | (Wire.read() << 8);
        y = Wire.read() | (Wire.read() << 8);
        z = Wire.read() | (Wire.read() << 8);
        return true;
    }

    // Heading in degrees, 0-360, from already-read X/Y raw values.
    float headingFromRaw(int16_t x, int16_t y) {
        float heading = atan2f((float)y, (float)x) * 180.0f / (float)M_PI;
        if (heading < 0) heading += 360.0f;
        lastHeading = heading;
        return heading;
    }

    // Heading in degrees, 0-360, tilt-uncompensated. Mount the module flat.
    float headingDeg() {
        int16_t x, y, z;
        if (!read(x, y, z)) return lastHeading;
        return headingFromRaw(x, y);
    }

private:
    static const int MODE_REG = 0x0A;
    static const int CONFIG_REG = 0x0B;
    static const int X_LSB_REG = 0x01;

    float lastHeading = 0.0f;

    void writeReg(uint8_t reg, uint8_t val) {
        Wire.beginTransmission(QMC5883P_ADDR);
        Wire.write(reg);
        Wire.write(val);
        Wire.endTransmission();
    }
};
