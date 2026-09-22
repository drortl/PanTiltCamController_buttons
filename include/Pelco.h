#pragma once
// Pelco-D controller over RS485 (UART): sends pan/tilt/stop/preset commands
// to the external pan-tilt head, and can also receive Pelco-D/P frames
// (auto-detected per frame) from another controller sharing the same bus.
//
// Pelco-D frame (7 bytes):  FF addr cmd1 cmd2 panSpeed tiltSpeed checksum
//   checksum = (addr+cmd1+cmd2+panSpeed+tiltSpeed) & 0xFF
// Pelco-P frame (8 bytes):  A0 addr cmd1 cmd2 panSpeed tiltSpeed AF checksum
//   checksum = addr ^ cmd1 ^ cmd2 ^ panSpeed ^ tiltSpeed ^ 0xAF
// cmd2 bits (shared by both): 0x02=pan right, 0x04=pan left, 0x08=tilt up, 0x10=tilt down
// "Set preset" command:  cmd1=0x00, cmd2=0x03, data1=0x00, data2=preset number
// "Call preset" command: cmd1=0x00, cmd2=0x07, data1=0x00, data2=preset number
// "Query position" (non-standard, see queryPositionDeg() below): cmd2=0x51 (pan) / 0x53 (tilt)
//
// NOTE: per this pan-tilt unit's manual, preset numbers 17-125 are reserved
// for built-in functions (limit scan, cruise tracks, guard position, self-
// check, etc.) rather than plain saved positions - see its manual before
// using setPreset()/callPreset() with a number in that range.

#include <Arduino.h>

struct PelcoCommand {
    uint8_t address = 0;
    bool panLeft = false;
    bool panRight = false;
    bool tiltUp = false;
    bool tiltDown = false;
    uint8_t panSpeed = 0;   // 0-63
    uint8_t tiltSpeed = 0;  // 0-63
};

class PelcoController {
public:
    void begin(HardwareSerial &serial, int rxPin, int txPin, int deRePin, uint32_t baud) {
        port = &serial;
        this->deRePin = deRePin;
        pinMode(deRePin, OUTPUT);
        digitalWrite(deRePin, LOW); // idle in receive mode
        port->begin(baud, SERIAL_8N1, rxPin, txPin);
    }

    // Call every loop(). Returns true and fills cmd when a valid frame arrived.
    bool poll(PelcoCommand &cmd) {
        while (port->available()) {
            uint8_t b = port->read();
            if (state == WAIT_SYNC) {
                if (b == 0xFF) { buf[0] = b; idx = 1; frameLen = 7; proto = PROTO_D; state = READING; }
                else if (b == 0xA0) { buf[0] = b; idx = 1; frameLen = 8; proto = PROTO_P; state = READING; }
                continue;
            }
            buf[idx++] = b;
            if (idx >= frameLen) {
                bool ok = (proto == PROTO_D) ? parseD(cmd) : parseP(cmd);
                state = WAIT_SYNC;
                idx = 0;
                if (ok) return true;
            }
        }
        return false;
    }

    // Continuous pan/tilt jog - the head moves until a sendStop() (or a new
    // sendMove()) is received. panSpeed/tiltSpeed are 0-63.
    void sendMove(uint8_t address, bool panLeft, bool panRight, bool tiltUp, bool tiltDown,
                  uint8_t panSpeed, uint8_t tiltSpeed) {
        uint8_t cmd2 = 0;
        if (panRight) cmd2 |= 0x02;
        if (panLeft)  cmd2 |= 0x04;
        if (tiltUp)   cmd2 |= 0x08;
        if (tiltDown) cmd2 |= 0x10;
        sendFrame(address, 0x00, cmd2, panSpeed, tiltSpeed);
    }

    void sendStop(uint8_t address) {
        sendFrame(address, 0x00, 0x00, 0x00, 0x00);
    }

    void callPreset(uint8_t address, uint8_t presetNum) {
        sendFrame(address, 0x00, 0x07, 0x00, presetNum);
    }

    void setPreset(uint8_t address, uint8_t presetNum) {
        sendFrame(address, 0x00, 0x03, 0x00, presetNum);
    }

    // Non-standard "query position" extension (cmd2 0x51 = pan, 0x53 =
    // tilt) - NOT part of the official Pelco-D spec, so most generic/
    // off-brand units won't answer it at all. Confirmed against this same
    // unit via the sibling PanTiltRS485Controller (USB-RS485 dongle) app:
    // it replies with a 7-byte Pelco-D frame whose cmd2 is 0x59 (report
    // pan) / 0x5B (report tilt), data1/data2 the position as a big-endian
    // 16-bit value in 0.01 degree units. Blocks up to timeoutMs waiting for
    // the reply; returns false on timeout or a malformed/unexpected frame.
    bool queryPositionDeg(uint8_t address, bool pan, float &degrees, uint32_t timeoutMs = 250) {
        uint8_t cmd2 = pan ? 0x51 : 0x53;
        sendFrame(address, 0x00, cmd2, 0x00, 0x00);

        uint8_t reply[7];
        uint8_t n = 0;
        unsigned long deadline = millis() + timeoutMs;
        while (n < sizeof(reply) && (long)(millis() - deadline) < 0) {
            if (port->available()) reply[n++] = port->read();
            else yield(); // release the CPU while waiting - a tight spin here can starve
                          // the watchdog if the head is slow to reply or not connected
        }
        if (n != sizeof(reply) || reply[0] != 0xFF) return false;

        uint8_t addr = reply[1], cmd1 = reply[2], rcmd2 = reply[3], data1 = reply[4], data2 = reply[5], checksum = reply[6];
        if (((addr + cmd1 + rcmd2 + data1 + data2) & 0xFF) != checksum) return false;
        if (rcmd2 != (pan ? 0x59 : 0x5B)) return false;

        degrees = ((data1 << 8) | data2) / 100.0f;
        return true;
    }

private:
    enum State { WAIT_SYNC, READING };
    enum Proto { PROTO_D, PROTO_P };

    HardwareSerial *port = nullptr;
    int deRePin = -1;
    State state = WAIT_SYNC;
    Proto proto = PROTO_D;
    uint8_t buf[8];
    uint8_t idx = 0;
    uint8_t frameLen = 7;

    static void fromCmd2(PelcoCommand &cmd, uint8_t cmd2) {
        cmd.panRight = cmd2 & 0x02;
        cmd.panLeft  = cmd2 & 0x04;
        cmd.tiltUp   = cmd2 & 0x08;
        cmd.tiltDown = cmd2 & 0x10;
    }

    bool parseD(PelcoCommand &cmd) {
        uint8_t addr = buf[1], cmd1 = buf[2], cmd2 = buf[3], panSpeed = buf[4], tiltSpeed = buf[5], checksum = buf[6];
        uint8_t sum = (addr + cmd1 + cmd2 + panSpeed + tiltSpeed) & 0xFF;
        if (sum != checksum) return false;
        cmd.address = addr;
        cmd.panSpeed = panSpeed;
        cmd.tiltSpeed = tiltSpeed;
        fromCmd2(cmd, cmd2);
        return true;
    }

    bool parseP(PelcoCommand &cmd) {
        uint8_t addr = buf[1], cmd1 = buf[2], cmd2 = buf[3], panSpeed = buf[4], tiltSpeed = buf[5], etx = buf[6], checksum = buf[7];
        if (etx != 0xAF) return false;
        uint8_t x = addr ^ cmd1 ^ cmd2 ^ panSpeed ^ tiltSpeed ^ etx;
        if (x != checksum) return false;
        cmd.address = addr;
        cmd.panSpeed = panSpeed;
        cmd.tiltSpeed = tiltSpeed;
        fromCmd2(cmd, cmd2);
        return true;
    }

    void sendFrame(uint8_t address, uint8_t cmd1, uint8_t cmd2, uint8_t data1, uint8_t data2) {
        uint8_t checksum = (address + cmd1 + cmd2 + data1 + data2) & 0xFF;
        uint8_t frame[7] = {0xFF, address, cmd1, cmd2, data1, data2, checksum};
        digitalWrite(deRePin, HIGH); // switch transceiver to transmit
        delayMicroseconds(10);
        port->write(frame, 7);
        port->flush(); // block until the bytes are physically on the wire
        delayMicroseconds(10);
        digitalWrite(deRePin, LOW); // back to receive
    }
};
