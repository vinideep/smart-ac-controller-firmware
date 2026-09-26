#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>

namespace ac::sensors {

struct PresenceReading {
    bool present = false;
    uint32_t duration_seconds = 0;
    uint32_t last_seen_ms = 0;
    const char* status = "idle";
};

class PresenceSensorDriver {
public:
    explicit PresenceSensorDriver(uint8_t pin, bool activeLow = false);

    void begin();
    bool update();

    void setActiveLow(bool activeLow);
    bool isActiveLow() const { return _activeLow; }

    const PresenceReading& getReading() const { return _reading; }
    bool isPresent() const { return _reading.present; }
    uint32_t getDurationSeconds() const { return _reading.duration_seconds; }

    void toJSON(JsonDocument& doc) const;

private:
    uint8_t _pin;
    bool _activeLow;
    bool _initialized = false;
    PresenceReading _reading;
    uint32_t _stateStartTime = 0;
    uint32_t _lastPollTime = 0;
    uint8_t _debounceCount = 0;
    bool _rawState = false;
};

} // namespace ac::sensors
