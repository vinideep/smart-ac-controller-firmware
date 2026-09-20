#include "presence_sensor.h"

namespace ac::sensors {

PresenceSensorDriver::PresenceSensorDriver(uint8_t pin, bool activeLow)
    : _pin(pin), _activeLow(activeLow) {}

void PresenceSensorDriver::begin() {
    if (!_initialized) {
        pinMode(_pin, _activeLow ? INPUT_PULLUP : INPUT);
        _stateStartTime = millis();
        _reading.last_seen_ms = millis();
        _initialized = true;
    }
}

bool PresenceSensorDriver::update() {
    if (!_initialized) begin();

    uint32_t now = millis();
    if (now - _lastPollTime < 50) { // 20Hz debounce sampling
        return false;
    }
    _lastPollTime = now;

    int pinLevel = digitalRead(_pin);
    bool detected = _activeLow ? (pinLevel == LOW) : (pinLevel == HIGH);

    if (detected == _rawState) {
        if (_debounceCount < 3) {
            _debounceCount++;
        }
    } else {
        _rawState = detected;
        _debounceCount = 0;
    }

    bool stateChanged = false;
    if (_debounceCount >= 3 && detected != _reading.present) {
        _reading.present = detected;
        _stateStartTime = now;
        stateChanged = true;
    }

    _reading.duration_seconds = (now - _stateStartTime) / 1000;
    if (_reading.present) {
        _reading.last_seen_ms = now;
        _reading.status = "ROOM_OCCUPIED";
    } else {
        _reading.status = "ROOM_EMPTY";
    }

    return stateChanged;
}

void PresenceSensorDriver::toJSON(JsonDocument& doc) const {
    doc["presence"] = _reading.present;
    doc["duration_seconds"] = _reading.duration_seconds;
    doc["last_seen_ms"] = _reading.last_seen_ms;
    doc["status"] = _reading.status;
}

} // namespace ac::sensors
