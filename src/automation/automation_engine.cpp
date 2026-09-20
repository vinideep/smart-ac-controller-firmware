#include "automation_engine.h"

namespace ac::automation {

AutomationEngine::AutomationEngine(control::ACController& ac,
                                   safety::SafetyManager& safety,
                                   sensors::PresenceSensorDriver& presence,
                                   DHTDriver& dht)
    : _ac(ac), _safety(safety), _presence(presence), _dht(dht) {}

void AutomationEngine::begin() {
    _lastEvalTime = millis();
}

void AutomationEngine::setEnabled(bool enable) {
    _config.enabled = enable;
    logEvent(enable ? "automation_enabled" : "automation_disabled", "user_toggle", _dht.getLatestReading().temperature_c);
}

void AutomationEngine::setTargetTemperature(float target) {
    if (target >= 16.0f && target <= 31.0f) {
        _config.targetTemperature = target;
    }
}

void AutomationEngine::setHysteresis(float hyst) {
    if (hyst >= 0.2f && hyst <= 5.0f) {
        _config.hysteresis = hyst;
    }
}

void AutomationEngine::setEmptyTimeoutSeconds(uint32_t seconds) {
    _config.emptyTimeoutSeconds = seconds;
}

void AutomationEngine::update() {
    if (!_config.enabled) return;

    uint32_t now = millis();
    if (now - _lastEvalTime < _config.evalIntervalMs) {
        return;
    }
    _lastEvalTime = now;

    const DHTReading& reading = _dht.getLatestReading();
    if (!reading.valid || reading.isStale(10000)) {
        return; // Don't make automation decisions on stale or invalid sensor data
    }

    float currentTemp = reading.temperature_c;
    bool isPresent = _presence.isPresent();
    uint32_t emptyDuration = isPresent ? 0 : _presence.getDurationSeconds();
    bool acOn = _safety.isAcPowered();

    // Rule 1: Empty room safety turn-off
    if (acOn && !isPresent && emptyDuration >= _config.emptyTimeoutSeconds) {
        const char* reason = nullptr;
        if (_safety.canTurnOff(now, reason)) {
            _ac.setPower(false, "automation_empty_room");
            _safety.recordPowerTransition(false, now);
            logEvent("power_off", "empty_room_timeout", currentTemp);
        }
        return;
    }

    // Rule 2: Occupied room cooling trigger
    if (!acOn && isPresent && currentTemp > (_config.targetTemperature + _config.hysteresis)) {
        const char* reason = nullptr;
        if (_safety.canTurnOn(now, reason)) {
            _ac.setTemperature((uint8_t)round(_config.targetTemperature), "automation_climate");
            _ac.setPower(true, "automation_climate");
            _safety.recordPowerTransition(true, now);
            logEvent("power_on", "temp_above_target_threshold", currentTemp);
        }
        return;
    }

    // Rule 3: Target reached in cooling mode
    if (acOn && isPresent && currentTemp < (_config.targetTemperature - _config.hysteresis)) {
        const char* reason = nullptr;
        if (_safety.canTurnOff(now, reason)) {
            _ac.setPower(false, "automation_target_reached");
            _safety.recordPowerTransition(false, now);
            logEvent("power_off", "target_temperature_reached", currentTemp);
        }
    }
}

void AutomationEngine::logEvent(const char* action, const char* reason, float currentTemp) {
    JsonDocument doc;
    doc["type"] = "automation_event";
    doc["action"] = action;
    doc["reason"] = reason;
    doc["current_temp"] = currentTemp;
    doc["target_temp"] = _config.targetTemperature;
    doc["hysteresis"] = _config.hysteresis;
    doc["timestamp_ms"] = millis();

    String out;
    serializeJson(doc, out);
    Serial.println(out);
}

void AutomationEngine::toJSON(JsonDocument& doc) const {
    doc["enabled"] = _config.enabled;
    doc["target_temp"] = _config.targetTemperature;
    doc["hysteresis"] = _config.hysteresis;
    doc["empty_timeout_s"] = _config.emptyTimeoutSeconds;
}

} // namespace ac::automation
