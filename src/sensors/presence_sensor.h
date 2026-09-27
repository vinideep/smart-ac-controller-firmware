#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>

namespace ac::sensors {

struct PresenceReading {
    bool present = false;
    uint32_t duration_seconds = 0;
    uint32_t last_seen_ms = 0;
    const char* status = "ROOM_EMPTY";
    float distance_m = 0.0f;
    uint8_t target_count = 0;
    const char* motion_state = "None";
    uint16_t moving_distance_cm = 0;
    uint8_t moving_energy = 0;
    uint16_t stationary_distance_cm = 0;
    uint8_t stationary_energy = 0;
};

class PresenceSensorDriver {
public:
    explicit PresenceSensorDriver(uint8_t pin, bool activeLow = false, int8_t rxPin = 16, int8_t txPin = 17, uint32_t baud = 115200);

    void begin();
    bool update();

    void setActiveLow(bool activeLow);
    bool isActiveLow() const { return _activeLow; }

    const PresenceReading& getReading() const { return _reading; }
    bool isPresent() const { return _reading.present; }
    uint32_t getDurationSeconds() const { return _reading.duration_seconds; }
    float getDistanceM() const { return _reading.distance_m; }
    uint8_t getTargetCount() const { return _reading.target_count; }
    const char* getMotionState() const { return _reading.motion_state; }
    bool isUartActive() const { return (_lastUartFrameTime > 0 && (millis() - _lastUartFrameTime < 2500)); }
    uint32_t getUartBytesReceived() const { return _totalUartBytes; }
    void printDebug(Print& out);

    void toJSON(JsonDocument& doc) const;

private:
    void processUart();
    bool parseFrame(const uint8_t* buf, size_t len);

    uint8_t _pin;
    bool _activeLow;
    int8_t _rxPin;
    int8_t _txPin;
    uint32_t _baud;
    bool _initialized = false;
    PresenceReading _reading;
    uint32_t _stateStartTime = 0;
    uint32_t _lastPollTime = 0;
    uint32_t _lastUartFrameTime = 0;
    uint32_t _totalUartBytes = 0;
    uint8_t _debounceCount = 0;
    bool _rawState = false;

    uint8_t _rxBuf[128];
    size_t _rxLen = 0;
};

} // namespace ac::sensors
