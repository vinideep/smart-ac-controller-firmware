#include "automation_engine.h"
#include <math.h>

#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
#include <Preferences.h>
#endif

namespace ac::automation {

AutomationEngine::AutomationEngine(control::ACController& ac,
                                   safety::SafetyManager& safety,
                                   sensors::PresenceSensorDriver& presence,
                                   DHTDriver& dht)
    : _ac(ac), _safety(safety), _presence(presence), _dht(dht) {}

void AutomationEngine::loadFromNVS() {
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
    Preferences prefs;
    if (prefs.begin("ac_auto", true)) {
        _config.enabled = prefs.getBool("en", _config.enabled);
        _config.targetTemperature = prefs.getFloat("target", _config.targetTemperature);
        _config.baseTargetTemp = prefs.getFloat("base_tgt", _config.baseTargetTemp);
        _config.hysteresis = prefs.getFloat("hyst", _config.hysteresis);
        _config.emptyTimeoutSeconds = prefs.getUInt("empty_s", _config.emptyTimeoutSeconds);
        _config.ecoDriftEnabled = prefs.getBool("eco_en", _config.ecoDriftEnabled);
        _config.ecoDriftTimeoutSeconds = prefs.getUInt("eco_s", _config.ecoDriftTimeoutSeconds);
        _config.psychrometricEnabled = prefs.getBool("psy_en", _config.psychrometricEnabled);
        _config.dryModeHumidityThreshold = prefs.getFloat("dry_th", _config.dryModeHumidityThreshold);
        _config.thermalBreachProtection = prefs.getBool("breach_en", _config.thermalBreachProtection);
        _config.presenceDetectionEnabled = prefs.getBool("pres_en", _config.presenceDetectionEnabled);
        prefs.end();
    }
#endif
}

void AutomationEngine::saveToNVS() {
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
    Preferences prefs;
    if (prefs.begin("ac_auto", false)) {
        prefs.putBool("en", _config.enabled);
        prefs.putFloat("target", _config.targetTemperature);
        prefs.putFloat("base_tgt", _config.baseTargetTemp);
        prefs.putFloat("hyst", _config.hysteresis);
        prefs.putUInt("empty_s", _config.emptyTimeoutSeconds);
        prefs.putBool("eco_en", _config.ecoDriftEnabled);
        prefs.putUInt("eco_s", _config.ecoDriftTimeoutSeconds);
        prefs.putBool("psy_en", _config.psychrometricEnabled);
        prefs.putFloat("dry_th", _config.dryModeHumidityThreshold);
        prefs.putBool("breach_en", _config.thermalBreachProtection);
        prefs.putBool("pres_en", _config.presenceDetectionEnabled);
        prefs.end();
    }
#endif
}

void AutomationEngine::begin() {
    _lastEvalTime = millis();
    _lastBreachSampleTime = millis();
    _breachBufCount = 0;
    _breachBufHead = 0;
    _turnedOffByVacancy = false;
    loadFromNVS();
}

void AutomationEngine::setEnabled(bool enable) {
    _config.enabled = enable;
    saveToNVS();
    logEvent(enable ? "automation_enabled" : "automation_disabled", "user_toggle", _dht.getLatestReading().temperature_c);
}

void AutomationEngine::setTargetTemperature(float target) {
    if (target >= 16.0f && target <= 31.0f) {
        _config.targetTemperature = target;
        _config.baseTargetTemp = target;
        saveToNVS();
    }
}

void AutomationEngine::setHysteresis(float hyst) {
    if (hyst >= 0.2f && hyst <= 5.0f) {
        _config.hysteresis = hyst;
        saveToNVS();
    }
}

void AutomationEngine::setEmptyTimeoutSeconds(uint32_t seconds) {
    _config.emptyTimeoutSeconds = seconds;
    saveToNVS();
}

void AutomationEngine::setEcoDriftEnabled(bool enable) {
    _config.ecoDriftEnabled = enable;
    saveToNVS();
}

void AutomationEngine::setEcoDriftTimeoutSeconds(uint32_t seconds) {
    _config.ecoDriftTimeoutSeconds = seconds;
    saveToNVS();
}

void AutomationEngine::setPsychrometricEnabled(bool enable) {
    _config.psychrometricEnabled = enable;
    saveToNVS();
}

void AutomationEngine::setDryModeHumidityThreshold(float threshold) {
    if (threshold >= 40.0f && threshold <= 90.0f) {
        _config.dryModeHumidityThreshold = threshold;
        saveToNVS();
    }
}

void AutomationEngine::setThermalBreachProtection(bool enable) {
    _config.thermalBreachProtection = enable;
    saveToNVS();
}

void AutomationEngine::setPresenceDetectionEnabled(bool enable) {
    _config.presenceDetectionEnabled = enable;
    saveToNVS();
    logEvent(enable ? "presence_guard_enabled" : "presence_guard_disabled", "config_update", _dht.getLatestReading().temperature_c);
}

void AutomationEngine::setCircadianSleep(bool enable, uint8_t pulldown, float ramp, float maxTemp) {
    _config.sleepConfig.enabled = enable;
    if (enable) {
        _config.sleepConfig.pulldownTemp = (pulldown >= 18 && pulldown <= 28) ? pulldown : 23;
        _config.sleepConfig.rampRatePerHr = (ramp >= 0.1f && ramp <= 2.0f) ? ramp : 0.5f;
        _config.sleepConfig.maxRampTemp = (maxTemp >= 20.0f && maxTemp <= 30.0f) ? maxTemp : 25.5f;
        _config.sleepConfig.sleepStartTimeMs = millis();
        _sleepStage = SleepStage::Pulldown;
        _config.targetTemperature = _config.sleepConfig.pulldownTemp;
        logEvent("sleep_schedule_started", "circadian_init", _dht.getLatestReading().temperature_c);
    } else {
        _sleepStage = SleepStage::Inactive;
        _config.targetTemperature = _config.baseTargetTemp;
        logEvent("sleep_schedule_cancelled", "user_toggle", _dht.getLatestReading().temperature_c);
    }
}

const char* AutomationEngine::getSleepStageStr() const {
    switch (_sleepStage) {
        case SleepStage::Pulldown: return "pulldown";
        case SleepStage::DeepSleepRamp: return "deep_sleep_ramp";
        case SleepStage::RemHold: return "rem_hold";
        case SleepStage::Wakeup: return "wakeup";
        case SleepStage::Inactive:
        default: return "inactive";
    }
}

const char* AutomationEngine::getPresenceTierStr() const {
    switch (_presenceTier) {
        case PresenceTier::EcoDrift: return "eco_drift";
        case PresenceTier::Vacant: return "vacant";
        case PresenceTier::WelcomeBack: return "welcome_back";
        case PresenceTier::Active:
        default: return "active";
    }
}

void AutomationEngine::recordBreachSample(float temp, uint32_t now) {
    if (now - _lastBreachSampleTime < 10000) {
        return; // Sample every 10 seconds
    }
    _lastBreachSampleTime = now;

    _breachBuffer[_breachBufHead].temp = temp;
    _breachBuffer[_breachBufHead].timeMs = now;
    _breachBufHead = (_breachBufHead + 1) % kBreachBufferSize;
    if (_breachBufCount < kBreachBufferSize) {
        _breachBufCount++;
    }
}

void AutomationEngine::evaluateThermalBreach(float currentTemp, uint32_t now, bool acOn) {
    if (!_config.thermalBreachProtection) {
        if (_thermalBreachActive) {
            _thermalBreachActive = false;
            _thermalBreachDelta = 0.0f;
        }
        return;
    }

    if (_breachBufCount < 4) return; // Need at least 4 samples (40s)

    bool cooling = acOn && (_ac.getState().mode == "cool");
    if (!cooling) {
        if (_thermalBreachActive) {
            _thermalBreachActive = false;
            _thermalBreachDelta = 0.0f;
        }
        return;
    }

    // Look for baseline sample within the 3-minute window (20s to 180s)
    float maxRise = 0.0f;
    bool foundInWindow = false;
    for (size_t i = 0; i < _breachBufCount; i++) {
        size_t idx = (_breachBufHead + kBreachBufferSize - 1 - i) % kBreachBufferSize;
        uint32_t ageMs = now - _breachBuffer[idx].timeMs;
        if (ageMs >= 20000 && ageMs <= _config.breachWindowMs) {
            foundInWindow = true;
            float rise = currentTemp - _breachBuffer[idx].temp;
            if (rise > maxRise) {
                maxRise = rise;
            }
        }
    }

    if (!foundInWindow) return;

    if (maxRise >= _config.breachRiseThreshold) {
        if (!_thermalBreachActive) {
            _thermalBreachActive = true;
            _thermalBreachDelta = maxRise;
            logEvent("thermal_breach_detected", "temp_rose_gt_1_2c_in_3m", currentTemp);

            // Thermal leak mitigation: boost blower to compensate
            if (_ac.getState().fanSpeed != "high") {
                _ac.setFanSpeed("high", "breach_mitigation");
            }
        } else {
            _thermalBreachDelta = maxRise;
        }
    } else if (_thermalBreachActive && maxRise < 0.3f) {
        // Cooled down back to normal trend
        _thermalBreachActive = false;
        _thermalBreachDelta = 0.0f;
        logEvent("thermal_breach_resolved", "thermal_trend_stabilized", currentTemp);
    }
}

void AutomationEngine::evaluateCircadianSleep(uint32_t now) {
    if (!_config.sleepConfig.enabled) return;

    uint32_t elapsedMs = now - _config.sleepConfig.sleepStartTimeMs;

    // Stage 1: Bedtime Pulldown (0 - 60 min)
    if (elapsedMs < 3600000) {
        _sleepStage = SleepStage::Pulldown;
        _config.targetTemperature = _config.sleepConfig.pulldownTemp;
    }
    // Stage 2: Deep Sleep Gradual Ramp (1 hr - 6 hrs = 3,600,000 to 21,600,000 ms)
    else if (elapsedMs < 21600000) {
        _sleepStage = SleepStage::DeepSleepRamp;
        float hoursInRamp = (elapsedMs - 3600000) / 3600000.0f;
        float ramped = _config.sleepConfig.pulldownTemp + (hoursInRamp * _config.sleepConfig.rampRatePerHr);
        if (ramped > _config.sleepConfig.maxRampTemp) ramped = _config.sleepConfig.maxRampTemp;
        _config.targetTemperature = ramped;
    }
    // Stage 3: REM Hold (6 hrs - 8 hrs = 21,600,000 to 28,800,000 ms)
    else if (elapsedMs < 28800000) {
        _sleepStage = SleepStage::RemHold;
        _config.targetTemperature = _config.sleepConfig.maxRampTemp;
    }
    // Stage 4: Wakeup (> 8 hrs)
    else {
        _sleepStage = SleepStage::Wakeup;
        const char* reason = nullptr;
        if (_safety.canTurnOff(now, reason)) {
            _ac.setPower(false, "circadian_wakeup_complete");
            _safety.recordPowerTransition(false, now);
            _config.sleepConfig.enabled = false;
            _sleepStage = SleepStage::Inactive;
            _config.targetTemperature = _config.baseTargetTemp;
            logEvent("circadian_sleep_ended", "full_night_schedule_completed", _dht.getLatestReading().temperature_c);
        }
    }
}

void AutomationEngine::update() {
    uint32_t now = millis();

    // 1. Maintain thermal breach tracking samples regardless of automation master toggle
    const DHTReading& reading = _dht.getLatestReading();
    if (reading.valid && !reading.isStale(10000)) {
        recordBreachSample(reading.temperature_c, now);
        evaluateThermalBreach(reading.temperature_c, now, _safety.isAcPowered());
    }

    if (!_config.enabled) return;

    if (now - _lastEvalTime < _config.evalIntervalMs) {
        return;
    }
    _lastEvalTime = now;

    if (!reading.valid || reading.isStale(10000)) {
        return; // Don't act on stale or invalid readings
    }

    float currentTemp = reading.temperature_c;
    float currentHum = reading.humidity_percent;
    float apparentTemp = _config.psychrometricEnabled ? reading.heat_index_c : currentTemp;
    bool isPresent = _presence.isPresent();
    uint32_t emptyDuration = isPresent ? 0 : _presence.getDurationSeconds();
    bool acOn = _safety.isAcPowered();
    bool sleepActive = _config.sleepConfig.enabled;
    bool effectivePresence = !_config.presenceDetectionEnabled || isPresent || sleepActive;

    // 2. Update Circadian Sleep Engine
    evaluateCircadianSleep(now);

    // 3. Multi-Tier Presence & Micro-Zoning
    if (!_config.presenceDetectionEnabled) {
        // Presence sensor requirement disabled: continuous climate control in active tier
        _presenceTier = PresenceTier::Active;
        _lastPresenceState = true;
    } else if (sleepActive) {
        // Nocturnal sleep active: occupant is present in bed (motionless); suspend vacant shutdown and eco-drift
        _presenceTier = PresenceTier::Active;
        _lastPresenceState = true;
    } else if (isPresent) {
        if (!_lastPresenceState || _presenceTier == PresenceTier::EcoDrift || _presenceTier == PresenceTier::Vacant) {
            // Welcome-Back Tier: Motion just detected after absence
            _presenceTier = PresenceTier::WelcomeBack;
            _config.targetTemperature = _config.baseTargetTemp;
            _turnedOffByVacancy = false;
            if (acOn) {
                uint8_t restored = (uint8_t)round(_config.baseTargetTemp);
                _ac.setState(true, restored, _ac.getState().mode, "auto", "welcome_back");
                _safety.recordCommandSent(now);
                logEvent("welcome_back_motion", "setpoint_restored", currentTemp);
            } else if (apparentTemp > _config.baseTargetTemp) {
                const char* reason = nullptr;
                if (_safety.canTurnOn(now, reason)) {
                    _ac.setState(true, (uint8_t)round(_config.baseTargetTemp), "cool", "auto", "welcome_back");
                    _safety.recordPowerTransition(true, now);
                    acOn = true;
                    logEvent("power_on", "welcome_back_restoration", currentTemp);
                }
            }
        } else {
            _presenceTier = PresenceTier::Active;
        }
        _lastPresenceState = true;
    } else {
        _lastPresenceState = false;
        if (emptyDuration < _config.ecoDriftTimeoutSeconds) {
            _presenceTier = PresenceTier::Active;
        } else if (emptyDuration < _config.emptyTimeoutSeconds) {
            // Eco-Drift Tier: 5 to 15 minutes unoccupied
            if (_config.ecoDriftEnabled) {
                if (_presenceTier != PresenceTier::EcoDrift) {
                    _presenceTier = PresenceTier::EcoDrift;
                    if (acOn) {
                        uint8_t ecoSetpoint = (uint8_t)min((float)31.0f, _config.baseTargetTemp + 1.0f);
                        _ac.setState(true, ecoSetpoint, _ac.getState().mode, "low", "eco_drift");
                        _safety.recordCommandSent(now);
                        logEvent("eco_drift_engaged", "unoccupied_5min_drift", currentTemp);
                    }
                }
            }
        } else {
            // Vacant Tier: > 15 minutes unoccupied -> Safe Graceful Shutdown
            _presenceTier = PresenceTier::Vacant;
            if (acOn) {
                const char* reason = nullptr;
                if (_safety.canTurnOff(now, reason)) {
                    _ac.setPower(false, "automation_vacant_timeout");
                    _safety.recordPowerTransition(false, now);
                    _turnedOffByVacancy = true;
                    logEvent("power_off", "vacant_room_timeout", currentTemp);
                }
                return;
            }
        }
    }

    // 4. Psychrometric Mode Arbitration
    // If humidity is high (>65%) but room temp is mild (21C to 27.5C), switch to DRY (dehumidify)
    if (_config.psychrometricEnabled && currentHum > _config.dryModeHumidityThreshold && currentTemp >= 21.0f && currentTemp <= 27.5f) {
        if (!acOn && effectivePresence) {
            const char* reason = nullptr;
            if (_safety.canTurnOn(now, reason)) {
                _ac.setState(true, _ac.getState().temperature, "dry", _ac.getState().fanSpeed, "psychro_humidity_arbitration");
                _safety.recordPowerTransition(true, now);
                acOn = true;
                logEvent("power_on", "high_humidity_dehumidify_mode", currentTemp);
            }
            return;
        } else if (acOn && _ac.getState().mode != "dry") {
            _ac.setMode("dry", "psychro_humidity_arbitration");
            _safety.recordCommandSent(now);
            logEvent("mode_switched_dry", "high_humidity_threshold_exceeded", currentTemp);
        }
    }
    // Switch back to COOL if heat index is high or temp is warm
    else if (_config.psychrometricEnabled && (apparentTemp > (_config.targetTemperature + _config.hysteresis) || currentTemp > 27.5f)) {
        if (acOn && _ac.getState().mode == "dry") {
            _ac.setMode("cool", "psychro_heat_arbitration");
            _safety.recordCommandSent(now);
            logEvent("mode_switched_cool", "heat_index_requires_cooling", currentTemp);
        }
    }

    // Dynamic setpoint tracking (Circadian sleep ramp / Target update while running)
    if (acOn && _presenceTier != PresenceTier::EcoDrift) {
        uint8_t desiredTemp = (uint8_t)round(_config.targetTemperature);
        if (_ac.getState().temperature != desiredTemp) {
            _ac.setTemperature(desiredTemp, _config.sleepConfig.enabled ? "circadian_ramp" : "target_update");
            _safety.recordCommandSent(now);
        }
    }

    // 5. Standard Closed-Loop Thermal Regulation (Heat-Index or Raw Temp)
    // Rule: Room hotter than target threshold -> turn ON
    if (!acOn && effectivePresence && apparentTemp > (_config.targetTemperature + _config.hysteresis)) {
        const char* reason = nullptr;
        if (_safety.canTurnOn(now, reason)) {
            _ac.setState(true, (uint8_t)round(_config.targetTemperature), "cool", _ac.getState().fanSpeed, "automation_climate");
            _safety.recordPowerTransition(true, now);
            logEvent("power_on", "heat_index_above_threshold", currentTemp);
        }
        return;
    }

    // Rule: Room colder than target threshold -> turn OFF
    if (acOn && effectivePresence && apparentTemp < (_config.targetTemperature - _config.hysteresis)) {
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
    doc["presence_tier"] = getPresenceTierStr();
    doc["sleep_stage"] = getSleepStageStr();
    doc["thermal_breach"] = _thermalBreachActive;
    doc["timestamp_ms"] = millis();

    String out;
    serializeJson(doc, out);
    Serial.println(out);
}

void AutomationEngine::toJSON(JsonDocument& doc) const {
    doc["enabled"] = _config.enabled;
    doc["target_temp"] = _config.targetTemperature;
    doc["base_target_temp"] = _config.baseTargetTemp;
    doc["hysteresis"] = _config.hysteresis;
    doc["empty_timeout_s"] = _config.emptyTimeoutSeconds;
    doc["eco_drift_enabled"] = _config.ecoDriftEnabled;
    doc["eco_drift_timeout_s"] = _config.ecoDriftTimeoutSeconds;
    doc["psychrometric_enabled"] = _config.psychrometricEnabled;
    doc["dry_mode_humidity_threshold"] = _config.dryModeHumidityThreshold;
    doc["thermal_breach_protection"] = _config.thermalBreachProtection;
    doc["thermal_breach"] = _thermalBreachActive;
    doc["thermal_breach_delta"] = _thermalBreachDelta;
    doc["presence_tier"] = getPresenceTierStr();
    doc["sleep_stage"] = getSleepStageStr();
    doc["sleep_enabled"] = _config.sleepConfig.enabled;
    doc["presence_detection_enabled"] = _config.presenceDetectionEnabled;
}

} // namespace ac::automation
