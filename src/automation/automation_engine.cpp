#include "automation_engine.h"
#include <math.h>
#include <time.h>
#include <sys/time.h>

#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
#include <Preferences.h>
#endif

namespace ac::automation {

AutomationEngine::AutomationEngine(control::ACController& ac,
                                   safety::SafetyManager& safety,
                                   sensors::PresenceSensorDriver& presence,
                                   DHTDriver& dht)
    : _ac(ac), _safety(safety), _presence(presence), _dht(dht) {}

void AutomationEngine::syncTime(uint32_t epochSec, int16_t tzOffsetMin) {
    if (epochSec > 1700000000) {
        timeval tv = { .tv_sec = (time_t)epochSec, .tv_usec = 0 };
        settimeofday(&tv, nullptr);
        char tzBuf[32];
        int hrs = tzOffsetMin / 60;
        int mins = abs(tzOffsetMin % 60);
        snprintf(tzBuf, sizeof(tzBuf), "UTC%+d:%02d", -hrs, mins);
        setenv("TZ", tzBuf, 1);
        tzset();
        Serial.printf("[TIME] Synced epoch %u with TZ offset %d mins\n", epochSec, tzOffsetMin);
    }
}

bool AutomationEngine::hasValidTime() const {
    time_t nowSec = time(nullptr);
    return (nowSec > 1700000000);
}

bool AutomationEngine::getLocalTime(uint8_t& outHour, uint8_t& outMinute, uint8_t& outSecond) const {
    time_t nowSec = time(nullptr);
    if (nowSec > 1700000000) {
        struct tm ti;
        localtime_r(&nowSec, &ti);
        outHour = ti.tm_hour;
        outMinute = ti.tm_min;
        outSecond = ti.tm_sec;
        return true;
    }
    outHour = 22; // Default to 10 PM
    outMinute = 0;
    outSecond = 0;
    return false;
}

uint8_t AutomationEngine::getLocalHour() const {
    uint8_t h, m, s;
    getLocalTime(h, m, s);
    return h;
}

uint8_t AutomationEngine::getLocalMinute() const {
    uint8_t h, m, s;
    getLocalTime(h, m, s);
    return m;
}

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
        _config.comfortIndexOptimization = prefs.getBool("comf_en", _config.comfortIndexOptimization);
        _config.dryModeHumidityThreshold = prefs.getFloat("dry_th", _config.dryModeHumidityThreshold);
        _config.thermalBreachProtection = prefs.getBool("breach_en", _config.thermalBreachProtection);
        _config.presenceDetectionEnabled = prefs.getBool("pres_en", _config.presenceDetectionEnabled);
        _config.continuousInverterMode = prefs.getBool("inv_mode", _config.continuousInverterMode);
        _config.nightCycle.enabled = prefs.getBool("night_en", _config.nightCycle.enabled);
        _config.nightCycle.targetTemp = prefs.getFloat("night_tgt", _config.nightCycle.targetTemp);
        if (_config.nightCycle.enabled) {
            _nightCycleStage = NightCycleStage::InitialPulldown;
            _config.nightCycle.stageStartTimeMs = millis();
        }
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
        prefs.putBool("comf_en", _config.comfortIndexOptimization);
        prefs.putFloat("dry_th", _config.dryModeHumidityThreshold);
        prefs.putBool("breach_en", _config.thermalBreachProtection);
        prefs.putBool("pres_en", _config.presenceDetectionEnabled);
        prefs.putBool("inv_mode", _config.continuousInverterMode);
        prefs.putBool("night_en", _config.nightCycle.enabled);
        prefs.putFloat("night_tgt", _config.nightCycle.targetTemp);
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

void AutomationEngine::setComfortIndexOptimization(bool enable) {
    _config.comfortIndexOptimization = enable;
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

void AutomationEngine::setContinuousInverterMode(bool enable) {
    _config.continuousInverterMode = enable;
    saveToNVS();
    logEvent(enable ? "inverter_mode_enabled" : "cycling_mode_enabled", "config_update", _dht.getLatestReading().temperature_c);
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

void AutomationEngine::setNightCycle(bool enable, float targetTemp, int8_t overrideHour, int8_t overrideMin) {
    _config.nightCycle.enabled = enable;
    if (targetTemp >= 18.0f && targetTemp <= 31.0f) {
        _config.nightCycle.targetTemp = targetTemp;
    }
    if (enable) {
        uint8_t curH = 0, curM = 0, curS = 0;
        bool hasTime = getLocalTime(curH, curM, curS);
        if (overrideHour >= 0 && overrideHour < 24) {
            curH = overrideHour;
            curM = (overrideMin >= 0 && overrideMin < 60) ? overrideMin : 0;
            hasTime = true;
        } else if (!hasTime) {
            curH = 22; // Default to 10 PM
            curM = 0;
        }

        _config.nightCycle.startHour = curH;
        _config.nightCycle.startMinute = curM;
        _config.nightCycle.stageStartTimeMs = millis();
        _config.nightCycle.preDawnTriggered = false;

        // Auto-adjust starting phase based on current time of the location
        if (curH >= 21 && curH < 23) {
            // 9 PM - 10:59 PM (e.g. 10 PM): Stage 1 initial pulldown to 27C
            _nightCycleStage = NightCycleStage::InitialPulldown;
        } else if (curH == 23) {
            // 11 PM: Room pulldown to 27C, then directly onto 40m pause and midnight
            _nightCycleStage = NightCycleStage::InitialPulldown;
        } else if (curH >= 0 && curH < 3) {
            // Midnight - 2:59 AM: Mid-night cooling to 27C with extended rest
            _nightCycleStage = NightCycleStage::Midnight_Cool27;
        } else if (curH >= 3 && curH < 5) {
            // 3:00 AM - 4:59 AM: Pre-dawn 20-minute burst window
            _nightCycleStage = NightCycleStage::PreDawn_Burst20m;
            _config.nightCycle.preDawnTriggered = true;
        } else if (curH >= 5 && curH < 8) {
            // 5:00 AM - 7:59 AM: Morning rest / completion
            _nightCycleStage = NightCycleStage::PreDawn_Pause2h;
        } else {
            _nightCycleStage = NightCycleStage::InitialPulldown;
        }

        const char* reason = nullptr;
        uint32_t now = millis();
        if (_nightCycleStage == NightCycleStage::InitialPulldown || 
            _nightCycleStage == NightCycleStage::Cycle2_Cool27 || 
            _nightCycleStage == NightCycleStage::Midnight_Cool27 || 
            _nightCycleStage == NightCycleStage::PreDawn_Burst20m) {
            if (_safety.canTurnOn(now, reason)) {
                uint8_t t = (uint8_t)round(_config.nightCycle.targetTemp);
                const char* fan = (_nightCycleStage == NightCycleStage::PreDawn_Burst20m || 
                                   _nightCycleStage == NightCycleStage::Midnight_Cool27) ? "low" : "auto";
                _ac.setState(true, t, "cool", fan, "night_cycle_start");
                _safety.recordPowerTransition(true, now);
            }
        } else {
            if (_safety.canTurnOff(now, reason)) {
                _ac.setPower(false, "night_cycle_pause_start");
                _safety.recordPowerTransition(false, now);
            }
        }
        logEvent("night_cycle_started", "time_calibrated_start", _dht.getLatestReading().temperature_c);
    } else {
        _nightCycleStage = NightCycleStage::Inactive;
        _config.targetTemperature = _config.baseTargetTemp;
        logEvent("night_cycle_stopped", "manual_override", _dht.getLatestReading().temperature_c);
    }
    saveToNVS();
}

const char* AutomationEngine::getNightCycleStageStr() const {
    switch (_nightCycleStage) {
        case NightCycleStage::InitialPulldown:        return "initial_pulldown";
        case NightCycleStage::Pause1_30m:             return "pause1_30m";
        case NightCycleStage::Cycle2_Cool27:          return "cycle2_cool27";
        case NightCycleStage::Pause2_40m:             return "pause2_40m";
        case NightCycleStage::Midnight_Cool27:        return "midnight_cool27";
        case NightCycleStage::Midnight_PauseExtended: return "midnight_pause_extended";
        case NightCycleStage::PreDawn_Burst20m:       return "predawn_burst20m";
        case NightCycleStage::PreDawn_Pause2h:        return "predawn_pause2h";
        case NightCycleStage::Completed:              return "completed";
        case NightCycleStage::Inactive:
        default:                                      return "inactive";
    }
}

uint32_t AutomationEngine::getNightCycleStageRemainingSec() const {
    if (!_config.nightCycle.enabled) return 0;
    uint32_t elapsedSec = (millis() - _config.nightCycle.stageStartTimeMs) / 1000;
    uint8_t curH = 0, curM = 0, curS = 0;
    bool hasClock = getLocalTime(curH, curM, curS);

    switch (_nightCycleStage) {
        case NightCycleStage::InitialPulldown:
            return elapsedSec < 3600 ? (3600 - elapsedSec) : 0;
        case NightCycleStage::Pause1_30m:
            return elapsedSec < _config.nightCycle.pause1DurationSec ? (_config.nightCycle.pause1DurationSec - elapsedSec) : 0;
        case NightCycleStage::Cycle2_Cool27:
            return elapsedSec < 2700 ? (2700 - elapsedSec) : 0;
        case NightCycleStage::Pause2_40m:
            return elapsedSec < _config.nightCycle.pause2DurationSec ? (_config.nightCycle.pause2DurationSec - elapsedSec) : 0;
        case NightCycleStage::Midnight_Cool27:
            return elapsedSec < 1800 ? (1800 - elapsedSec) : 0;
        case NightCycleStage::Midnight_PauseExtended: {
            if (hasClock && curH < 3) {
                return (uint32_t)((2 - curH) * 3600 + (59 - curM) * 60 + (60 - curS));
            }
            return elapsedSec < _config.nightCycle.midnightPauseSec ? (_config.nightCycle.midnightPauseSec - elapsedSec) : 0;
        }
        case NightCycleStage::PreDawn_Burst20m:
            return elapsedSec < _config.nightCycle.preDawnBurstSec ? (_config.nightCycle.preDawnBurstSec - elapsedSec) : 0;
        case NightCycleStage::PreDawn_Pause2h:
            return elapsedSec < _config.nightCycle.preDawnPauseSec ? (_config.nightCycle.preDawnPauseSec - elapsedSec) : 0;
        default:
            return 0;
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

void AutomationEngine::evaluateNightCycle(uint32_t now) {
    if (!_config.nightCycle.enabled) return;

    const DHTReading& reading = _dht.getLatestReading();
    float currentTemp = reading.valid ? reading.temperature_c : 27.0f;
    uint32_t stageElapsedMs = now - _config.nightCycle.stageStartTimeMs;
    uint32_t stageElapsedSec = stageElapsedMs / 1000;
    const char* reason = nullptr;
    uint8_t targetByte = (uint8_t)round(_config.nightCycle.targetTemp);

    uint8_t curH = 0, curM = 0, curS = 0;
    bool hasClock = getLocalTime(curH, curM, curS);

    // Freeze protection safeguard: if room drops below 18C, turn off immediately
    if (_safety.isAcPowered() && currentTemp < 18.0f) {
        if (_safety.canTurnOff(now, reason)) {
            _ac.setPower(false, "freeze_protection_cutoff");
            _safety.recordPowerTransition(false, now);
            logEvent("power_off", "night_cycle_freeze_cutoff", currentTemp);
        }
        return;
    }

    // Location Clock Auto-Adjustment: Trigger 3 AM - 5 AM window burst if not yet run
    if (hasClock && curH >= 3 && curH < 5 && !_config.nightCycle.preDawnTriggered) {
        if (_nightCycleStage != NightCycleStage::PreDawn_Burst20m && 
            _nightCycleStage != NightCycleStage::PreDawn_Pause2h &&
            _nightCycleStage != NightCycleStage::Completed) {
            if (_safety.canTurnOn(now, reason)) {
                _ac.setState(true, targetByte, "cool", "low", "night_cycle_predawn_burst_clock");
                _safety.recordPowerTransition(true, now);
                _nightCycleStage = NightCycleStage::PreDawn_Burst20m;
                _config.nightCycle.stageStartTimeMs = now;
                _config.nightCycle.preDawnTriggered = true;
                logEvent("night_cycle_predawn_burst", "wall_clock_3am_window_burst", currentTemp);
                return;
            }
        }
    }

    switch (_nightCycleStage) {
        case NightCycleStage::InitialPulldown: {
            // Stage 1: Turn on AC to bring room temperature to 27C initially (max 60m safety guard)
            bool targetAchieved = reading.valid && (currentTemp <= _config.nightCycle.targetTemp);
            bool maxTimeout = (stageElapsedSec >= 3600);
            if (targetAchieved || maxTimeout) {
                if (_safety.canTurnOff(now, reason)) {
                    _ac.setPower(false, "night_cycle_target_27c_reached");
                    _safety.recordPowerTransition(false, now);
                    _nightCycleStage = NightCycleStage::Pause1_30m;
                    _config.nightCycle.stageStartTimeMs = now;
                    logEvent("night_cycle_pause1", "target_27c_reached_turn_off_30m", currentTemp);
                }
            } else if (!_safety.isAcPowered()) {
                if (_safety.canTurnOn(now, reason)) {
                    _ac.setState(true, targetByte, "cool", "auto", "night_cycle_pulldown");
                    _safety.recordPowerTransition(true, now);
                }
            }
            break;
        }

        case NightCycleStage::Pause1_30m: {
            // Stage 2: Turn off AC for 30 minutes
            bool timeElapsed = (stageElapsedSec >= _config.nightCycle.pause1DurationSec);
            bool overheatEarly = reading.valid && (currentTemp >= _config.nightCycle.targetTemp + 2.5f) && (stageElapsedSec >= 600);
            if (timeElapsed || overheatEarly) {
                if (_safety.canTurnOn(now, reason)) {
                    _ac.setState(true, targetByte, "cool", "auto", "night_cycle_cool2_start");
                    _safety.recordPowerTransition(true, now);
                    _nightCycleStage = NightCycleStage::Cycle2_Cool27;
                    _config.nightCycle.stageStartTimeMs = now;
                    logEvent("night_cycle_cool2", "pause1_30m_complete_bring_to_27c", currentTemp);
                }
            }
            break;
        }

        case NightCycleStage::Cycle2_Cool27: {
            // Stage 3: Bring temperature back to 27C (max 45m safety guard)
            bool targetAchieved = reading.valid && (currentTemp <= _config.nightCycle.targetTemp);
            bool maxTimeout = (stageElapsedSec >= 2700);
            if (targetAchieved || maxTimeout) {
                if (_safety.canTurnOff(now, reason)) {
                    _ac.setPower(false, "night_cycle_cycle2_reached");
                    _safety.recordPowerTransition(false, now);
                    _nightCycleStage = NightCycleStage::Pause2_40m;
                    _config.nightCycle.stageStartTimeMs = now;
                    logEvent("night_cycle_pause2", "cycle2_reached_turn_off_40m", currentTemp);
                }
            } else if (!_safety.isAcPowered()) {
                if (_safety.canTurnOn(now, reason)) {
                    _ac.setState(true, targetByte, "cool", "auto", "night_cycle_cool2");
                    _safety.recordPowerTransition(true, now);
                }
            }
            break;
        }

        case NightCycleStage::Pause2_40m: {
            // Stage 4: Turn off AC for 40 minutes (auto-advances to midnight if clock is >= 00:00)
            bool timeElapsed = (stageElapsedSec >= _config.nightCycle.pause2DurationSec);
            bool overheatEarly = reading.valid && (currentTemp >= _config.nightCycle.targetTemp + 2.5f) && (stageElapsedSec >= 600);
            bool midnightClockReached = hasClock && (curH >= 0 && curH < 3 && stageElapsedSec >= 900);
            if (timeElapsed || overheatEarly || midnightClockReached) {
                if (_safety.canTurnOn(now, reason)) {
                    _ac.setState(true, targetByte, "cool", "low", "night_cycle_midnight_start");
                    _safety.recordPowerTransition(true, now);
                    _nightCycleStage = NightCycleStage::Midnight_Cool27;
                    _config.nightCycle.stageStartTimeMs = now;
                    logEvent("night_cycle_midnight_cool", "pause2_complete_start_midnight", currentTemp);
                }
            }
            break;
        }

        case NightCycleStage::Midnight_Cool27: {
            // Stage 5: Mid-night maintenance cool to 27C with low blower
            bool targetAchieved = reading.valid && (currentTemp <= _config.nightCycle.targetTemp);
            bool maxTimeout = (stageElapsedSec >= 1800);
            if (targetAchieved || maxTimeout) {
                if (_safety.canTurnOff(now, reason)) {
                    _ac.setPower(false, "night_cycle_midnight_reached");
                    _safety.recordPowerTransition(false, now);
                    _nightCycleStage = NightCycleStage::Midnight_PauseExtended;
                    _config.nightCycle.stageStartTimeMs = now;

                    // Calibrate pause so it lands at 03:00 AM based on local clock
                    if (hasClock && curH < 3) {
                        uint32_t secUntil3am = (uint32_t)((2 - curH) * 3600 + (59 - curM) * 60 + (60 - curS));
                        _config.nightCycle.midnightPauseSec = max((uint32_t)2700, secUntil3am);
                    } else {
                        _config.nightCycle.midnightPauseSec = 4500;
                    }

                    logEvent("night_cycle_midnight_pause", "midnight_reached_extended_pause", currentTemp);
                }
            } else if (!_safety.isAcPowered()) {
                if (_safety.canTurnOn(now, reason)) {
                    _ac.setState(true, targetByte, "cool", "low", "night_cycle_midnight_cool");
                    _safety.recordPowerTransition(true, now);
                }
            }
            break;
        }

        case NightCycleStage::Midnight_PauseExtended: {
            // Stage 6: Mid-night extended OFF period (reduced cooling demand)
            bool timeElapsed = (stageElapsedSec >= _config.nightCycle.midnightPauseSec);
            bool clockHit3am = hasClock && (curH >= 3);
            bool overheatEarly = reading.valid && (currentTemp >= _config.nightCycle.targetTemp + 2.5f) && (stageElapsedSec >= 1800);
            if (timeElapsed || clockHit3am || overheatEarly) {
                if (_safety.canTurnOn(now, reason)) {
                    _ac.setState(true, targetByte, "cool", "low", "night_cycle_predawn_start");
                    _safety.recordPowerTransition(true, now);
                    _nightCycleStage = NightCycleStage::PreDawn_Burst20m;
                    _config.nightCycle.stageStartTimeMs = now;
                    _config.nightCycle.preDawnTriggered = true;
                    logEvent("night_cycle_predawn_burst", "midnight_pause_done_start_20m_burst", currentTemp);
                }
            }
            break;
        }

        case NightCycleStage::PreDawn_Burst20m: {
            // Stage 7: 3 AM - 5 AM window: Turn ON for 20 minutes (1200s)
            bool burstComplete = (stageElapsedSec >= _config.nightCycle.preDawnBurstSec);
            if (burstComplete) {
                if (_safety.canTurnOff(now, reason)) {
                    _ac.setPower(false, "night_cycle_predawn_burst_done");
                    _safety.recordPowerTransition(false, now);
                    _nightCycleStage = NightCycleStage::PreDawn_Pause2h;
                    _config.nightCycle.stageStartTimeMs = now;
                    logEvent("night_cycle_predawn_pause", "20m_burst_done_turn_off_2h", currentTemp);
                }
            } else if (!_safety.isAcPowered()) {
                if (_safety.canTurnOn(now, reason)) {
                    _ac.setState(true, targetByte, "cool", "low", "night_cycle_predawn_burst");
                    _safety.recordPowerTransition(true, now);
                }
            }
            break;
        }

        case NightCycleStage::PreDawn_Pause2h: {
            // Stage 8: Turn OFF for 2 hours (7200s) or until morning wakeup (>= 06:00 AM)
            bool pauseComplete = (stageElapsedSec >= _config.nightCycle.preDawnPauseSec);
            bool morningWakeup = hasClock && (curH >= 6 && curH < 20);
            if (pauseComplete || morningWakeup) {
                _nightCycleStage = NightCycleStage::Completed;
                _config.nightCycle.enabled = false;
                _nightCycleStage = NightCycleStage::Inactive;
                _config.targetTemperature = _config.baseTargetTemp;
                logEvent("night_cycle_completed", "full_night_cycle_finished", currentTemp);
            }
            break;
        }

        default:
            break;
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
    bool sleepActive = _config.sleepConfig.enabled || _config.nightCycle.enabled;
    bool effectivePresence = !_config.presenceDetectionEnabled || isPresent || sleepActive;

    // 2. Update Night Sleep Cycle Automation
    if (_config.nightCycle.enabled) {
        evaluateNightCycle(now);
        return;
    }

    // 3. Update Circadian Sleep Engine
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

    // 5. Freeze Protection: Safety cutoff if room temperature drops below 18C
    if (acOn && currentTemp < 18.0f) {
        const char* reason = nullptr;
        if (_safety.canTurnOff(now, reason)) {
            _ac.setPower(false, "freeze_protection_cutoff");
            _safety.recordPowerTransition(false, now);
            logEvent("power_off", "room_below_freeze_limit", currentTemp);
        }
        return;
    }

    // 6. Standard Closed-Loop Thermal Regulation (Heat-Index or Raw Temp)
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

    // Rule: Room colder than target threshold
    // In continuous inverter mode (default): Keep AC running! Inverter compressor idles down to maintain setpoint.
    // In legacy cycling mode: Cut master power when target is achieved.
    if (acOn && effectivePresence && apparentTemp < (_config.targetTemperature - _config.hysteresis)) {
        if (!_config.continuousInverterMode) {
            const char* reason = nullptr;
            if (_safety.canTurnOff(now, reason)) {
                _ac.setPower(false, "automation_target_reached");
                _safety.recordPowerTransition(false, now);
                logEvent("power_off", "target_temperature_reached", currentTemp);
            }
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
    doc["comfort_index_optimization"] = _config.comfortIndexOptimization;
    doc["continuous_inverter_mode"] = _config.continuousInverterMode;
    doc["night_cycle_enabled"] = _config.nightCycle.enabled;
    doc["night_cycle_stage"] = (uint8_t)_nightCycleStage;
    doc["night_cycle_stage_str"] = getNightCycleStageStr();
    doc["night_cycle_target_temp"] = _config.nightCycle.targetTemp;
    doc["night_cycle_stage_remaining_s"] = getNightCycleStageRemainingSec();
    doc["night_cycle_start_hour"] = _config.nightCycle.startHour;
    doc["night_cycle_start_min"] = _config.nightCycle.startMinute;

    uint8_t curH = 0, curM = 0, curS = 0;
    bool validClock = getLocalTime(curH, curM, curS);
    doc["clock_synced"] = validClock;
    char timeStr[16];
    snprintf(timeStr, sizeof(timeStr), "%02u:%02u:%02u", curH, curM, curS);
    doc["local_time_str"] = timeStr;
    doc["local_hour"] = curH;
    doc["local_minute"] = curM;
}

} // namespace ac::automation
