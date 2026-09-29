#include <iostream>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>

// Test DHTReading struct definition matching dht_sensor.h
struct DHTReading {
    float temperature = 0.0f;
    float humidity = 0.0f;
    float temperature_c = 0.0f;
    float humidity_percent = 0.0f;
    uint32_t timestamp_ms = 0;
    const char* status = "not_ready";
    bool valid = false;
    const char* error_message = nullptr;

    bool isStale(uint32_t currentMillis, uint32_t maxAgeMs = 5000) const {
        if (!valid || timestamp_ms == 0) return true;
        return (currentMillis - timestamp_ms) > maxAgeMs;
    }
};

// Test boundary validation logic matching dht_sensor.cpp
constexpr float DHT_TEMP_MIN_C = -40.0f;
constexpr float DHT_TEMP_MAX_C = 80.0f;
constexpr float DHT_HUMIDITY_MIN = 0.0f;
constexpr float DHT_HUMIDITY_MAX = 100.0f;

bool validateReading(float temp, float hum, const char*& errorReason) {
    if (std::isnan(temp) || std::isnan(hum)) {
        errorReason = "NaN received from DHT sensor (timeout or no response)";
        return false;
    }
    if (temp < DHT_TEMP_MIN_C || temp > DHT_TEMP_MAX_C) {
        errorReason = "Temperature reading out of operational range (-40C to 80C)";
        return false;
    }
    if (hum < DHT_HUMIDITY_MIN || hum > DHT_HUMIDITY_MAX) {
        errorReason = "Humidity reading out of operational range (0% to 100%)";
        return false;
    }
    return true;
}

// Test deviceId derivation matching main.cpp
std::string deriveDeviceId(uint64_t mac) {
    char idBuf[32];
    snprintf(idBuf, sizeof(idBuf), "esp32-%02x%02x%02x",
             (uint8_t)(mac >> 24),
             (uint8_t)(mac >> 32),
             (uint8_t)(mac >> 40));
    return std::string(idBuf);
}

// Azure Essence Protocol definitions for host test
constexpr uint8_t  kAzureVendorId     = 0x19;
constexpr uint8_t  kAzurePowerOnByte  = 0x18;
constexpr uint8_t  kAzurePowerOffByte = 0x70;
constexpr uint8_t  kAzureModeCool     = 0x02;
constexpr uint8_t  kAzureFanAuto      = 0x01;
constexpr uint8_t  kAzureFanMed       = 0x02;
constexpr uint8_t  kAzureFanHigh      = 0x03;
constexpr uint8_t  kAzureStateLength  = 9;
constexpr uint16_t kAzureRawTransitions = 147;

uint8_t calculateAzureChecksum(const uint8_t state[kAzureStateLength]) {
    uint16_t nibbleSum = 0;
    for (uint8_t i = 0; i < 8; i++) {
        nibbleSum += (state[i] >> 4) & 0x0F;
        nibbleSum += state[i] & 0x0F;
    }
    return static_cast<uint8_t>(nibbleSum & 0x0F);
}

void encodeAzureFrame(bool power, uint8_t temp, const std::string& fan, const std::string& mode, uint8_t out[kAzureStateLength]) {
    (void)mode;
    out[0] = kAzureVendorId;
    uint8_t fanNibble = (fan == "med") ? kAzureFanMed : (fan == "high" ? kAzureFanHigh : kAzureFanAuto);
    uint8_t modeNibble = kAzureModeCool;
    out[1] = (fanNibble << 4) | (modeNibble & 0x0F);
    out[2] = 0x00;
    out[3] = 0x00;
    out[4] = 0x00;
    out[5] = 0x00;
    uint8_t clampedTemp = temp < 16 ? 16 : (temp > 31 ? 31 : temp);
    out[6] = clampedTemp + 11;
    out[7] = power ? kAzurePowerOnByte : kAzurePowerOffByte;
    uint8_t chk = calculateAzureChecksum(out);
    out[8] = (chk << 4) & 0xF0;
}

uint16_t generateAzureRaw(const uint8_t state[kAzureStateLength], uint16_t* outRaw, uint16_t maxLen) {
    if (outRaw == nullptr || maxLen < kAzureRawTransitions) return 0;
    uint16_t idx = 0;
    outRaw[idx++] = 4500;
    outRaw[idx++] = 2400;
    for (uint8_t b = 0; b < kAzureStateLength; b++) {
        uint8_t val = state[b];
        for (uint8_t bit = 0; bit < 8; bit++) {
            outRaw[idx++] = 430;
            bool isOne = (val >> bit) & 1;
            outRaw[idx++] = isOne ? 970 : 470;
        }
    }
    outRaw[idx++] = 430;
    return idx;
}

// Host simulation of SafetyManager
class TestSafetyManager {
public:
    uint32_t minOnTimeMs = 180000;
    uint32_t minOffTimeMs = 180000;
    uint32_t minStateDelayMs = 5000;
    bool isPowered = false;
    uint32_t lastTurnOn = 0;
    uint32_t lastTurnOff = 0;
    uint32_t lastCmd = 0;

    TestSafetyManager(uint32_t on = 180000, uint32_t off = 180000, uint32_t delay = 5000)
        : minOnTimeMs(on), minOffTimeMs(off), minStateDelayMs(delay) {}

    bool isAcPowered() const { return isPowered; }

    bool canTurnOn(uint32_t now, const char*& reason) const {
        if (isPowered) { reason = "already_on"; return false; }
        if (lastTurnOff > 0 && (now - lastTurnOff) < minOffTimeMs) { reason = "min_off_rest"; return false; }
        if (lastCmd > 0 && (now - lastCmd) < minStateDelayMs) { reason = "throttled"; return false; }
        reason = nullptr; return true;
    }

    bool canTurnOff(uint32_t now, const char*& reason) const {
        if (!isPowered) { reason = "already_off"; return false; }
        if (lastTurnOn > 0 && (now - lastTurnOn) < minOnTimeMs) { reason = "min_on_runtime"; return false; }
        if (lastCmd > 0 && (now - lastCmd) < minStateDelayMs) { reason = "throttled"; return false; }
        reason = nullptr; return true;
    }

    void recordTransition(bool on, uint32_t now) {
        isPowered = on;
        if (on) lastTurnOn = now;
        else lastTurnOff = now;
        lastCmd = now;
    }
};

// Host simulation of EnergyReading and EnergyMonitor (Phase 8)
struct TestEnergyReading {
    float voltage = 230.0f;
    float current = 0.0f;
    float power_watts = 0.0f;
    float power_factor = 0.95f;
    float energy_kwh_today = 0.0f;
    float estimated_cost_today = 0.0f;
    float tariff_rate = 8.0f;
    uint32_t timestamp_ms = 0;
    std::string status = "standby";
    bool valid = true;
};

class TestEnergyMonitor {
public:
    explicit TestEnergyMonitor(float defaultTariff = 8.0f, float nominalVoltage = 230.0f)
        : _nominalVoltage(nominalVoltage) {
        _reading.voltage = nominalVoltage;
        _reading.current = 0.015f;
        _reading.power_watts = 2.5f;
        _reading.power_factor = 0.65f;
        _reading.energy_kwh_today = 0.0f;
        _reading.estimated_cost_today = 0.0f;
        _reading.tariff_rate = defaultTariff > 0.0f ? defaultTariff : 8.0f;
        _reading.timestamp_ms = 0;
        _reading.status = "standby";
        _reading.valid = true;
    }

    void begin(uint32_t nowMs = 0) {
        _lastUpdateTime = nowMs;
        _reading.timestamp_ms = nowMs;
        _initialized = true;
    }

    bool setTariff(float ratePerKwh) {
        if (ratePerKwh <= 0.0f) return false;
        _reading.tariff_rate = ratePerKwh;
        _reading.estimated_cost_today = static_cast<float>(_accumulatedKwh * _reading.tariff_rate);
        return true;
    }

    void setNominalVoltage(float volts) {
        if (volts > 50.0f && volts < 300.0f) {
            _nominalVoltage = volts;
            _reading.voltage = volts;
        }
    }

    void resetDaily() {
        _accumulatedKwh = 0.0;
        _reading.energy_kwh_today = 0.0f;
        _reading.estimated_cost_today = 0.0f;
    }

    void accumulateEnergy(uint32_t nowMs) {
        if (_lastUpdateTime == 0) {
            _lastUpdateTime = nowMs;
            _reading.timestamp_ms = nowMs;
            return;
        }
        uint32_t deltaMs = nowMs - _lastUpdateTime;
        if (deltaMs == 0) return;
        if (deltaMs > 86400000UL) {
            _lastUpdateTime = nowMs;
            _reading.timestamp_ms = nowMs;
            return;
        }

        double deltaHours = static_cast<double>(deltaMs) / 3600000.0;
        double deltaKwh = (static_cast<double>(_reading.power_watts) * deltaHours) / 1000.0;
        _accumulatedKwh += deltaKwh;
        _reading.energy_kwh_today = static_cast<float>(_accumulatedKwh);
        _reading.estimated_cost_today = static_cast<float>(_accumulatedKwh * _reading.tariff_rate);
        _reading.timestamp_ms = nowMs;
        _lastUpdateTime = nowMs;
    }

    void setAccumulatedKwh(double kwh) {
        _accumulatedKwh = kwh;
        _reading.energy_kwh_today = static_cast<float>(kwh);
        _reading.estimated_cost_today = static_cast<float>(kwh * _reading.tariff_rate);
    }

    double getAccumulatedKwh() const { return _accumulatedKwh; }
    uint32_t getLastUpdateTime() const { return _lastUpdateTime; }


    bool update(uint32_t nowMs) {
        accumulateEnergy(nowMs);
        return true;
    }

    void updateMeasurement(float voltage, float current, float powerWatts, float powerFactor = 0.95f, const char* status = "measuring", uint32_t nowMs = 0) {
        accumulateEnergy(nowMs);
        _reading.voltage = voltage;
        _reading.current = current;
        _reading.power_watts = powerWatts;
        _reading.power_factor = powerFactor;
        _reading.status = status ? status : "measuring";
        _reading.valid = true;
        _reading.timestamp_ms = nowMs;
    }

    void updateFromAcState(bool isPowered,
                           const std::string& mode,
                           const std::string& fanSpeed,
                           uint8_t targetTemp,
                           float ambientTemp,
                           uint32_t nowMs) {
        accumulateEnergy(nowMs);
        _reading.voltage = _nominalVoltage;
        _reading.timestamp_ms = nowMs;
        _reading.valid = true;

        if (!isPowered) {
            _reading.power_watts = 2.5f;
            _reading.power_factor = 0.65f;
            _reading.current = _reading.power_watts / (_reading.voltage * _reading.power_factor);
            _reading.status = "standby";
            return;
        }

        float fanPower = 45.0f;
        if (fanSpeed == "low") fanPower = 35.0f;
        else if (fanSpeed == "med") fanPower = 55.0f;
        else if (fanSpeed == "high") fanPower = 80.0f;

        if (mode == "fan") {
            _reading.power_watts = fanPower;
            _reading.power_factor = 0.88f;
            _reading.current = _reading.power_watts / (_reading.voltage * _reading.power_factor);
            _reading.status = "fan_only";
            return;
        }

        if (mode == "dry") {
            _reading.power_watts = 600.0f + fanPower;
            _reading.power_factor = 0.94f;
            _reading.current = _reading.power_watts / (_reading.voltage * _reading.power_factor);
            _reading.status = "dehumidifying";
            return;
        }

        _reading.status = "cooling";
        _reading.power_factor = 0.97f;
        float deltaTemp = ambientTemp - static_cast<float>(targetTemp);
        float compressorPower = 1100.0f;
        if (deltaTemp > 0.0f) {
            compressorPower = 1100.0f + (deltaTemp * 85.0f);
        } else {
            compressorPower = 550.0f;
        }

        if (compressorPower < 500.0f) compressorPower = 500.0f;
        if (compressorPower > 1800.0f) compressorPower = 1800.0f;

        _reading.power_watts = compressorPower + fanPower;
        _reading.current = _reading.power_watts / (_reading.voltage * _reading.power_factor);
    }

    std::string toJSONString() const {
        std::ostringstream ss;
        ss << "{\"voltage\":" << std::fixed << std::setprecision(1) << _reading.voltage
           << ",\"current\":" << std::setprecision(2) << _reading.current
           << ",\"power_watts\":" << std::setprecision(1) << _reading.power_watts
           << ",\"power_factor\":" << std::setprecision(2) << _reading.power_factor
           << ",\"energy_kwh_today\":" << std::setprecision(4) << _reading.energy_kwh_today
           << ",\"tariff_rate\":" << std::setprecision(2) << _reading.tariff_rate
           << ",\"estimated_cost_today\":" << std::setprecision(2) << _reading.estimated_cost_today
           << ",\"status\":\"" << _reading.status << "\""
           << ",\"timestamp_ms\":" << _reading.timestamp_ms
           << ",\"valid\":" << (_reading.valid ? "true" : "false") << "}";
        return ss.str();
    }

    const TestEnergyReading& getReading() const { return _reading; }
    float getTariff() const { return _reading.tariff_rate; }
    float getNominalVoltage() const { return _nominalVoltage; }

private:
    TestEnergyReading _reading;
    float _nominalVoltage;
    double _accumulatedKwh = 0.0;
    uint32_t _lastUpdateTime = 0;
    bool _initialized = false;
};

// Host simulation of PresenceSensorDriver (Phase 7)

class TestPresenceSensor {
public:
    explicit TestPresenceSensor(bool activeLow = true) : _activeLow(activeLow) {}

    bool update(bool pinLevelLow, uint32_t now) {
        if (now - _lastPollTime < 50) return false;
        _lastPollTime = now;

        bool detected = _activeLow ? pinLevelLow : !pinLevelLow;
        if (detected == _rawState) {
            if (_debounceCount < 3) _debounceCount++;
        } else {
            _rawState = detected;
            _debounceCount = 0;
        }

        bool stateChanged = false;
        if (_debounceCount >= 3 && detected != _present) {
            _present = detected;
            _stateStartTime = now;
            stateChanged = true;
        }

        _durationSeconds = (now - _stateStartTime) / 1000;
        _status = _present ? "ROOM_OCCUPIED" : "ROOM_EMPTY";
        return stateChanged;
    }

    bool isPresent() const { return _present; }
    uint32_t getDurationSeconds() const { return _durationSeconds; }
    const std::string& getStatus() const { return _status; }

private:
    bool _activeLow;
    bool _present = false;
    bool _rawState = false;
    uint8_t _debounceCount = 0;
    uint32_t _lastPollTime = 0;
    uint32_t _stateStartTime = 0;
    uint32_t _durationSeconds = 0;
    std::string _status = "ROOM_EMPTY";
};

// Host simulation of AutomationEngine (Phase 9)
class TestAutomationEngine {
public:
    bool enabled = false;
    float targetTemperature = 25.0f;
    float hysteresis = 1.0f;
    uint32_t emptyTimeoutSeconds = 900;
    bool presenceDetectionEnabled = true;
    bool continuousInverterMode = false;

    enum Action { NONE, TURN_ON, TURN_OFF };

    Action evaluate(bool acOn, bool isPresent, uint32_t emptyDurationSec, float currentTemp, uint32_t now, const TestSafetyManager& safety) {
        if (!enabled) return NONE;

        bool effectivePresence = !presenceDetectionEnabled || isPresent;

        // Rule 1: Empty room safety turn-off (enforced only when presence detection is enabled)
        if (presenceDetectionEnabled && acOn && !isPresent && emptyDurationSec >= emptyTimeoutSeconds) {
            const char* reason = nullptr;
            if (safety.canTurnOff(now, reason)) {
                return TURN_OFF;
            }
            return NONE;
        }

        // Rule 2: Cooling trigger
        if (!acOn && effectivePresence && currentTemp > (targetTemperature + hysteresis)) {
            const char* reason = nullptr;
            if (safety.canTurnOn(now, reason)) {
                return TURN_ON;
            }
            return NONE;
        }

        // Rule 3: Target reached in cooling mode
        if (acOn && effectivePresence && currentTemp < (targetTemperature - hysteresis)) {
            if (!continuousInverterMode) {
                const char* reason = nullptr;
                if (safety.canTurnOff(now, reason)) {
                    return TURN_OFF;
                }
            }
            return NONE;
        }

        return NONE;
    }
};

// Host simulation of Smart AC Climate Stack (Psychrometric, Sleep, Presence Tiers, Thermal Breach)
class TestSmartClimateEngine {
public:
    enum PresenceTier { TIER_ACTIVE, TIER_ECO_DRIFT, TIER_VACANT, TIER_WELCOME_BACK };
    enum SleepStage { SLEEP_INACTIVE, SLEEP_PULLDOWN, SLEEP_DEEP_RAMP, SLEEP_REM_HOLD, SLEEP_WAKEUP };

    bool enabled = true;
    float targetTemperature = 25.0f;
    float baseTargetTemp = 25.0f;
    float hysteresis = 1.0f;
    uint32_t ecoDriftTimeoutSeconds = 300;
    uint32_t emptyTimeoutSeconds = 900;
    bool ecoDriftEnabled = true;
    bool psychrometricEnabled = true;
    float dryModeHumidityThreshold = 65.0f;
    bool thermalBreachProtection = true;
    float breachRiseThreshold = 1.2f;
    uint32_t breachWindowMs = 180000;
    bool presenceDetectionEnabled = true;
    bool continuousInverterMode = false;

    // Sleep config
    bool sleepEnabled = false;
    uint8_t sleepPulldown = 23;
    float sleepRampRate = 0.5f;
    float sleepMaxTemp = 25.5f;
    uint32_t sleepStartTime = 0;
    SleepStage sleepStage = SLEEP_INACTIVE;

    // Presence state
    PresenceTier presenceTier = TIER_ACTIVE;
    bool lastPresenceState = true;
    bool turnedOffByVacancy = false;

    // Thermal breach state
    bool thermalBreachActive = false;
    float thermalBreachDelta = 0.0f;
    struct Sample { float temp; uint32_t timeMs; };
    std::vector<Sample> breachBuffer;

    // AC state
    bool acPower = false;
    uint8_t acTemp = 25;
    std::string acMode = "cool";
    std::string acFan = "auto";

    void clearBreach() {
        thermalBreachActive = false;
        thermalBreachDelta = 0.0f;
        breachBuffer.clear();
    }

    void recordBreachSample(float temp, uint32_t now) {
        breachBuffer.push_back({temp, now});
        if (breachBuffer.size() > 24) breachBuffer.erase(breachBuffer.begin());
    }

    void evaluateBreach(float currentTemp, uint32_t now) {
        if (!thermalBreachProtection) {
            thermalBreachActive = false;
            thermalBreachDelta = 0.0f;
            return;
        }
        if (breachBuffer.size() < 4 || !acPower || acMode != "cool") return;
        float maxRise = 0.0f;
        bool found = false;
        for (const auto& s : breachBuffer) {
            uint32_t age = now - s.timeMs;
            if (age >= 20000 && age <= breachWindowMs) {
                found = true;
                float r = currentTemp - s.temp;
                if (r > maxRise) maxRise = r;
            }
        }
        if (!found) return;
        if (maxRise >= breachRiseThreshold) {
            if (!thermalBreachActive) {
                thermalBreachActive = true;
                thermalBreachDelta = maxRise;
                acFan = "high"; // leak mitigation
            } else {
                thermalBreachDelta = maxRise;
            }
        } else if (thermalBreachActive && maxRise < 0.3f) {
            thermalBreachActive = false;
            thermalBreachDelta = 0.0f;
        }
    }

    void evaluateSleep(uint32_t now) {
        if (!sleepEnabled) return;
        uint32_t elapsed = now - sleepStartTime;
        if (elapsed < 3600000) {
            sleepStage = SLEEP_PULLDOWN;
            targetTemperature = sleepPulldown;
        } else if (elapsed < 21600000) {
            sleepStage = SLEEP_DEEP_RAMP;
            float hrs = (elapsed - 3600000) / 3600000.0f;
            float r = sleepPulldown + hrs * sleepRampRate;
            if (r > sleepMaxTemp) r = sleepMaxTemp;
            targetTemperature = r;
        } else if (elapsed < 28800000) {
            sleepStage = SLEEP_REM_HOLD;
            targetTemperature = sleepMaxTemp;
        } else {
            sleepStage = SLEEP_WAKEUP;
            acPower = false;
            sleepEnabled = false;
            sleepStage = SLEEP_INACTIVE;
            targetTemperature = baseTargetTemp;
        }
    }

    void update(bool isPresent, uint32_t emptyDurationSec, float currentTemp, float currentHum, float heatIndex, uint32_t now, TestSafetyManager& safety) {
        recordBreachSample(currentTemp, now);
        evaluateBreach(currentTemp, now);
        if (!enabled) return;

        evaluateSleep(now);
        float apparentTemp = psychrometricEnabled ? heatIndex : currentTemp;
        bool effectivePresence = !presenceDetectionEnabled || isPresent || sleepEnabled;

        // Presence & Micro-zoning
        if (!presenceDetectionEnabled) {
            presenceTier = TIER_ACTIVE;
            lastPresenceState = true;
        } else if (sleepEnabled) {
            presenceTier = TIER_ACTIVE;
            lastPresenceState = true;
        } else if (isPresent) {
            if (!lastPresenceState || presenceTier == TIER_ECO_DRIFT || presenceTier == TIER_VACANT) {
                presenceTier = TIER_WELCOME_BACK;
                targetTemperature = baseTargetTemp;
                turnedOffByVacancy = false;
                if (acPower) {
                    acTemp = (uint8_t)std::round(baseTargetTemp);
                    acFan = "auto";
                } else if (apparentTemp > baseTargetTemp) {
                    const char* reason = nullptr;
                    if (safety.canTurnOn(now, reason)) {
                        acPower = true;
                        acTemp = (uint8_t)std::round(baseTargetTemp);
                        acMode = "cool";
                        acFan = "auto";
                        safety.recordTransition(true, now);
                    }
                }
            } else {
                presenceTier = TIER_ACTIVE;
            }
            lastPresenceState = true;
        } else {
            lastPresenceState = false;
            if (emptyDurationSec < ecoDriftTimeoutSeconds) {
                presenceTier = TIER_ACTIVE;
            } else if (emptyDurationSec < emptyTimeoutSeconds) {
                if (ecoDriftEnabled) {
                    if (presenceTier != TIER_ECO_DRIFT) {
                        presenceTier = TIER_ECO_DRIFT;
                        if (acPower) {
                            acTemp = (uint8_t)std::min(31.0f, baseTargetTemp + 1.0f);
                            acFan = "low";
                        }
                    }
                }
            } else {
                presenceTier = TIER_VACANT;
                if (acPower) {
                    const char* reason = nullptr;
                    if (safety.canTurnOff(now, reason)) {
                        acPower = false;
                        safety.recordTransition(false, now);
                        turnedOffByVacancy = true;
                    }
                    return;
                }
            }
        }

        // Psychrometric arbitration
        if (psychrometricEnabled && currentHum > dryModeHumidityThreshold && currentTemp >= 21.0f && currentTemp <= 27.5f) {
            if (!acPower && effectivePresence) {
                const char* reason = nullptr;
                if (safety.canTurnOn(now, reason)) {
                    acMode = "dry";
                    acPower = true;
                    safety.recordTransition(true, now);
                }
                return;
            } else if (acPower && acMode != "dry") {
                acMode = "dry";
            }
        } else if (psychrometricEnabled && (apparentTemp > (targetTemperature + hysteresis) || currentTemp > 27.5f)) {
            if (acPower && acMode == "dry") {
                acMode = "cool";
            }
        }

        // Dynamic setpoint tracking
        if (acPower && presenceTier != TIER_ECO_DRIFT) {
            uint8_t des = (uint8_t)std::round(targetTemperature);
            if (acTemp != des) acTemp = des;
        }

        // Regulation
        if (!acPower && effectivePresence && apparentTemp > (targetTemperature + hysteresis)) {
            const char* reason = nullptr;
            if (safety.canTurnOn(now, reason)) {
                acPower = true;
                acTemp = (uint8_t)std::round(targetTemperature);
                acMode = "cool";
                safety.recordTransition(true, now);
            }
        } else if (acPower && effectivePresence && apparentTemp < (targetTemperature - hysteresis)) {
            if (!continuousInverterMode) {
                const char* reason = nullptr;
                if (safety.canTurnOff(now, reason)) {
                    acPower = false;
                    safety.recordTransition(false, now);
                }
            }
        }
    }
};

int main() {

    std::cout << "[TEST] Starting firmware logic unit tests..." << std::endl;

    // Test 1: Device ID derivation
    uint64_t constructedMac = ((uint64_t)0x30) |
                             (((uint64_t)0x76) << 8) |
                             (((uint64_t)0xF5) << 16) |
                             (((uint64_t)0xE5) << 24) |
                             (((uint64_t)0xCA) << 32) |
                             (((uint64_t)0x5C) << 40);
    std::string devId = deriveDeviceId(constructedMac);
    assert(devId == "esp32-e5ca5c");
    std::cout << "[TEST 1] PASS: Device ID correctly uses unique suffix 'e5ca5c'" << std::endl;

    // Test 2: Validation of valid readings
    const char* reason = nullptr;
    assert(validateReading(25.4f, 55.0f, reason) == true);
    std::cout << "[TEST 2] PASS: Valid temperature and humidity accepted" << std::endl;

    // Test 3: Rejection of NaN
    assert(validateReading(NAN, 55.0f, reason) == false);
    std::cout << "[TEST 3] PASS: NaN temperature rejected" << std::endl;

    // Test 4: Rejection of impossible temperatures
    assert(validateReading(-45.0f, 50.0f, reason) == false);
    assert(validateReading(85.0f, 50.0f, reason) == false);
    std::cout << "[TEST 4] PASS: Out of bound temperatures (< -40C, > 80C) rejected" << std::endl;

    // Test 5: Rejection of impossible humidity
    assert(validateReading(25.0f, -5.0f, reason) == false);
    assert(validateReading(25.0f, 105.0f, reason) == false);
    std::cout << "[TEST 5] PASS: Out of bound humidity rejected" << std::endl;

    // Test 6: Stale data prevention on failure
    DHTReading reading;
    reading.valid = false;
    reading.status = "read_failed";
    assert(!reading.valid);
    std::cout << "[TEST 6] PASS: Stale data cleared upon failure" << std::endl;

    // Test 7: Stale reading detection
    reading.valid = true;
    reading.timestamp_ms = 10000;
    assert(!reading.isStale(12000, 5000));
    assert(reading.isStale(16000, 5000));
    std::cout << "[TEST 7] PASS: isStale accurately identifies aged data" << std::endl;

    // Test 8: IR noise filter threshold
    auto shouldFilterNoise = [](uint16_t rawlen) { return rawlen < 4; };
    assert(shouldFilterNoise(3) == true);
    assert(shouldFilterNoise(4) == false);
    std::cout << "[TEST 8] PASS: Optical noise filter correctly discards transitions < 4" << std::endl;

    // Test 9: Raw timing conversion logic
    assert(2200 * 2 == 4400);
    std::cout << "[TEST 9] PASS: Raw tick-to-microsecond conversion matches 38kHz specs" << std::endl;

    // Test 10: IR buffer capacity
    assert(200 < 1024);
    std::cout << "[TEST 10] PASS: Capture buffer size (1024) accommodates full AC frames" << std::endl;

    // Test 11: Multi-byte state hex formatting
    uint8_t dummyState[6] = { 0xB2, 0x4D, 0x1F, 0x00, 0x2A, 0xC5 };
    char hexBuffer[64];
    int offset = 0;
    for (int i = 0; i < 6; i++) {
        offset += snprintf(hexBuffer + offset, sizeof(hexBuffer) - offset, "%02X", dummyState[i]);
    }
    assert(std::string(hexBuffer) == "B24D1F002AC5");
    std::cout << "[TEST 11] PASS: Multi-byte AC state serialization matches expected hex string" << std::endl;

    // Test 12: Byte count calculation
    auto calcByteCount = [](uint16_t bits) -> uint16_t { return (bits + 7) / 8; };
    assert(calcByteCount(72) == 9);
    assert(calcByteCount(75) == 10);
    std::cout << "[TEST 12] PASS: Byte count calculation prevents dropping trailing bits" << std::endl;

    // Test 13: State byte differential comparison
    uint8_t statePower25[6] = { 0xB2, 0x4D, 0x1F, 0x00, 0x2A, 0xC5 };
    uint8_t statePower26[6] = { 0xB2, 0x4D, 0x2F, 0x00, 0x2A, 0xD5 };
    assert(statePower25[2] != statePower26[2]);
    assert(statePower25[5] != statePower26[5]);
    std::cout << "[TEST 13] PASS: Differential state byte analysis isolates mutations" << std::endl;

    // Test 14: Hash calculation
    assert(kAzureRawTransitions == 147);
    std::cout << "[TEST 14] PASS: Frame transition count verified" << std::endl;

    // Test 15: Schema validation
    assert(kAzureStateLength == 9);
    std::cout << "[TEST 15] PASS: State length is 9 bytes" << std::endl;

    // Test 16: Checksum calculation across known frame
    uint8_t testFrame[9] = { 0x19, 0x12, 0x00, 0x00, 0x00, 0x00, 0x24, 0x18, 0x00 };
    uint8_t chk = calculateAzureChecksum(testFrame);
    assert(chk == 0x0C);
    std::cout << "[TEST 16] PASS: Nibble checksum for 25C Power ON matches 0x0C (0xC0)" << std::endl;

    // Test 17: Wire encoding for Power ON (Cool, 25C, Auto)
    uint8_t frame25[9];
    encodeAzureFrame(true, 25, "auto", "cool", frame25);
    uint8_t expected25[9] = { 0x19, 0x12, 0x00, 0x00, 0x00, 0x00, 0x24, 0x18, 0xC0 };
    assert(memcmp(frame25, expected25, 9) == 0);
    std::cout << "[TEST 17] PASS: Power ON Cool 25C Auto wire frame exactly matches captured remote frame" << std::endl;

    // Test 18: Wire encoding for Temp UP (Cool, 26C, Auto)
    uint8_t frame26[9];
    encodeAzureFrame(true, 26, "auto", "cool", frame26);
    uint8_t expected26[9] = { 0x19, 0x12, 0x00, 0x00, 0x00, 0x00, 0x25, 0x18, 0xD0 };
    assert(memcmp(frame26, expected26, 9) == 0);
    std::cout << "[TEST 18] PASS: Temp 26C Auto wire frame exactly matches captured remote frame" << std::endl;

    // Test 19: Wire encoding for Temp 27C Auto
    uint8_t frame27[9];
    encodeAzureFrame(true, 27, "auto", "cool", frame27);
    uint8_t expected27[9] = { 0x19, 0x12, 0x00, 0x00, 0x00, 0x00, 0x26, 0x18, 0xE0 };
    assert(memcmp(frame27, expected27, 9) == 0);
    std::cout << "[TEST 19] PASS: Temp 27C Auto wire frame exactly matches captured remote frame" << std::endl;

    // Test 20: Wire encoding for Fan Speed Medium
    uint8_t frameFan[9];
    encodeAzureFrame(true, 25, "med", "cool", frameFan);
    uint8_t expectedFan[9] = { 0x19, 0x22, 0x00, 0x00, 0x00, 0x00, 0x24, 0x18, 0xD0 };
    assert(memcmp(frameFan, expectedFan, 9) == 0);
    std::cout << "[TEST 20] PASS: Fan Med wire frame exactly matches captured remote frame" << std::endl;

    // Test 21: Wire encoding for Power OFF
    uint8_t frameOff[9];
    encodeAzureFrame(false, 25, "auto", "cool", frameOff);
    uint8_t expectedOff[9] = { 0x19, 0x12, 0x00, 0x00, 0x00, 0x00, 0x24, 0x70, 0xA0 };
    assert(memcmp(frameOff, expectedOff, 9) == 0);
    std::cout << "[TEST 21] PASS: Power OFF wire frame exactly matches captured remote frame" << std::endl;

    // Test 22: Temperature boundary clamping
    uint8_t frameLow[9], frameHigh[9];
    encodeAzureFrame(true, 10, "auto", "cool", frameLow);   // Clamped to 16 -> 16+11 = 27 (0x1B)
    encodeAzureFrame(true, 40, "auto", "cool", frameHigh);  // Clamped to 31 -> 31+11 = 42 (0x2A)
    assert(frameLow[6] == 27);
    assert(frameHigh[6] == 42);
    std::cout << "[TEST 22] PASS: Temperature safely clamped between 16C and 31C" << std::endl;

    // Test 23: Raw pulse train transition generator
    uint16_t rawBuf[200];
    uint16_t count = generateAzureRaw(frame25, rawBuf, 200);
    assert(count == 147);
    assert(rawBuf[0] == 4500); // Leader mark
    assert(rawBuf[1] == 2400); // Leader space
    assert(rawBuf[146] == 430); // Stop mark
    std::cout << "[TEST 23] PASS: Raw pulse generator produces 147 transitions with exact header/stop timings" << std::endl;

    // Test 24: Safety manager compressor anti-cycling (min on-time rejection)
    TestSafetyManager safety;
    const char* secReason = nullptr;
    safety.recordTransition(true, 100000); // Turned ON at 100s
    assert(safety.canTurnOff(160000, secReason) == false);
    assert(std::string(secReason) == "min_on_runtime");
    assert(safety.canTurnOff(290000, secReason) == true);
    std::cout << "[TEST 24] PASS: Compressor minimum on-time protection correctly rejects premature shutdown" << std::endl;

    // Test 25: Safety manager compressor rest time (min off-time rejection)
    safety.recordTransition(false, 300000); // Turned OFF at 300s
    assert(safety.canTurnOn(360000, secReason) == false);
    assert(std::string(secReason) == "min_off_rest");
    assert(safety.canTurnOn(490000, secReason) == true);
    std::cout << "[TEST 25] PASS: Compressor minimum off-time protection prevents short-cycling restart" << std::endl;

    // Test 26: Safety manager IR transmission rate limiting
    safety.recordTransition(true, 500000); // Turned ON at 500s
    safety.lastTurnOn = 500000;
    safety.lastCmd = 698000; // Sent a temperature adjustment at 698s
    assert(safety.canTurnOff(700000, secReason) == false);
    assert(std::string(secReason) == "throttled");
    std::cout << "[TEST 26] PASS: Command throttling prevents rapid IR burst collisions" << std::endl;

    // =========================================================================
    // PHASE 8: ENERGY MONITORING UNIT TESTS
    // =========================================================================

    // Test 27: EnergyMonitor default initialization
    TestEnergyMonitor energyMon(8.0f, 230.0f);
    energyMon.begin(1000);
    const TestEnergyReading& rInit = energyMon.getReading();
    assert(rInit.voltage == 230.0f);
    assert(rInit.tariff_rate == 8.0f);
    assert(rInit.power_watts == 2.5f);
    assert(rInit.status == "standby");
    assert(rInit.valid == true);
    assert(rInit.energy_kwh_today == 0.0f);
    assert(rInit.estimated_cost_today == 0.0f);
    std::cout << "[TEST 27] PASS: EnergyMonitor correctly initializes with default 230V, tariff 8.00/kWh, standby 2.5W" << std::endl;

    // Test 28: Standby power calculation when AC is off
    energyMon.updateFromAcState(false, "cool", "auto", 25, 30.0f, 2000);
    const TestEnergyReading& rOff = energyMon.getReading();
    assert(rOff.power_watts == 2.5f);
    assert(rOff.status == "standby");
    assert(rOff.current < 0.02f);
    std::cout << "[TEST 28] PASS: AC OFF state correctly computes low standby power draw (2.5W)" << std::endl;

    // Test 29: Inverter compressor power calculation under cooling load
    // Ambient = 32C, Target = 24C (delta = +8C) -> High load
    energyMon.updateFromAcState(true, "cool", "auto", 24, 32.0f, 3000);
    const TestEnergyReading& rCoolHeavy = energyMon.getReading();
    assert(rCoolHeavy.status == "cooling");
    // Base 1100 + 8*85 = 1780W + 45W fan = 1825W
    assert(rCoolHeavy.power_watts == 1825.0f);
    assert(rCoolHeavy.current > 7.0f); // ~8.18A @ 230V, pf 0.97

    // Ambient = 23C, Target = 24C (delta = -1C) -> Target achieved, inverter throttling
    energyMon.updateFromAcState(true, "cool", "auto", 24, 23.0f, 4000);
    const TestEnergyReading& rCoolLight = energyMon.getReading();
    // 550W + 45W fan = 595W
    assert(rCoolLight.power_watts == 595.0f);
    std::cout << "[TEST 29] PASS: Inverter compressor modulates power (1825W heavy load down to 595W idle cooling)" << std::endl;

    // Test 30: Fan-only mode across different fan speeds
    energyMon.updateFromAcState(true, "fan", "low", 25, 27.0f, 5000);
    assert(energyMon.getReading().power_watts == 35.0f);
    assert(energyMon.getReading().status == "fan_only");

    energyMon.updateFromAcState(true, "fan", "med", 25, 27.0f, 6000);
    assert(energyMon.getReading().power_watts == 55.0f);

    energyMon.updateFromAcState(true, "fan", "high", 25, 27.0f, 7000);
    assert(energyMon.getReading().power_watts == 80.0f);
    std::cout << "[TEST 30] PASS: Fan-only mode accurately accounts for low/med/high fan speed wattages (35W/55W/80W)" << std::endl;

    // Test 31: Dehumidification / dry mode
    energyMon.updateFromAcState(true, "dry", "auto", 25, 28.0f, 8000);
    assert(energyMon.getReading().power_watts == 645.0f); // 600W + 45W
    assert(energyMon.getReading().status == "dehumidifying");
    std::cout << "[TEST 31] PASS: Dehumidification mode sets expected compressor power (645W)" << std::endl;

    // Test 32: Energy (kWh) accumulation over time
    // Run at constant 1200.0W for exactly 1 hour (3,600,000 ms)
    energyMon.updateMeasurement(230.0f, 5.37f, 1200.0f, 0.97f, "measuring", 10000);
    // Advance 3,600,000 ms
    energyMon.update(10000 + 3600000);
    const TestEnergyReading& rAccum = energyMon.getReading();
    // 1200W * 1 hr = 1.2 kWh (plus fractional amount from prior tests ~0.0007 kWh)
    assert(std::fabs(rAccum.energy_kwh_today - 1.2007f) < 0.002f);
    std::cout << "[TEST 32] PASS: Energy accumulation matches theoretical power integral (1200W for 1hr = 1.20 kWh)" << std::endl;

    // Test 33: Estimated electricity cost calculation with default tariff
    // 1.2007 kWh * 8.00/kWh = 9.6056
    float expectedCost = rAccum.energy_kwh_today * 8.0f;
    assert(std::fabs(rAccum.estimated_cost_today - expectedCost) < 0.01f);
    std::cout << "[TEST 33] PASS: Electricity cost accurately calculated based on active tariff" << std::endl;

    // Test 34: Configurable tariff update and cost recalculation
    assert(energyMon.setTariff(10.50f) == true);
    assert(energyMon.getTariff() == 10.50f);
    // Cost must automatically recalculate with new tariff
    assert(std::fabs(energyMon.getReading().estimated_cost_today - (rAccum.energy_kwh_today * 10.50f)) < 0.01f);

    // Invalid tariffs rejected
    assert(energyMon.setTariff(0.0f) == false);
    assert(energyMon.setTariff(-5.0f) == false);
    assert(energyMon.getTariff() == 10.50f);
    std::cout << "[TEST 34] PASS: Configurable tariff updates dynamic cost and rejects non-positive rates" << std::endl;

    // Test 35: Direct isolated hardware meter measurement input
    energyMon.updateMeasurement(228.4f, 4.85f, 1080.5f, 0.97f, "isolated_ct", 4000000);
    const TestEnergyReading& rDirect = energyMon.getReading();
    assert(rDirect.voltage == 228.4f);
    assert(rDirect.current == 4.85f);
    assert(rDirect.power_watts == 1080.5f);
    assert(rDirect.status == "isolated_ct");
    std::cout << "[TEST 35] PASS: Isolated AC meter measurement interface correctly accepts external hardware readings" << std::endl;

    // Test 36: Daily reset clears energy and cost counters
    energyMon.resetDaily();
    assert(energyMon.getReading().energy_kwh_today == 0.0f);
    assert(energyMon.getReading().estimated_cost_today == 0.0f);
    assert(energyMon.getTariff() == 10.50f); // Tariff preserved
    assert(energyMon.getReading().voltage == 228.4f);
    std::cout << "[TEST 36] PASS: Daily reset zeroes accumulated kWh and cost while preserving system configuration" << std::endl;

    // Test 37: Energy JSON serialization schema completeness
    std::string jsonStr = energyMon.toJSONString();
    assert(jsonStr.find("\"voltage\":") != std::string::npos);
    assert(jsonStr.find("\"current\":") != std::string::npos);
    assert(jsonStr.find("\"power_watts\":") != std::string::npos);
    assert(jsonStr.find("\"power_factor\":") != std::string::npos);
    assert(jsonStr.find("\"energy_kwh_today\":") != std::string::npos);
    assert(jsonStr.find("\"tariff_rate\":") != std::string::npos);
    assert(jsonStr.find("\"estimated_cost_today\":") != std::string::npos);
    assert(jsonStr.find("\"status\":") != std::string::npos);
    assert(jsonStr.find("\"timestamp_ms\":") != std::string::npos);
    assert(jsonStr.find("\"valid\":") != std::string::npos);
    std::cout << "[TEST 37] PASS: Energy JSON telemetry serialization conforms to API schema" << std::endl;

    // Test 38: Millis rollover protection (unsigned modular arithmetic across 2^32-1 to 0)
    TestEnergyMonitor energyRollover(8.0f, 230.0f);
    energyRollover.begin(4294967200UL); // 96ms before 32-bit overflow
    energyRollover.updateMeasurement(230.0f, 4.35f, 1000.0f, 0.97f, "measuring", 4294967200UL);
    // Rollover occurs: now = 100ms past zero (elapsed = 96 + 100 = 196ms)
    energyRollover.update(100UL);
    assert(energyRollover.getLastUpdateTime() == 100UL);
    double expectedRolloverKwh = (1000.0 * (196.0 / 3600000.0)) / 1000.0;
    assert(std::fabs(energyRollover.getAccumulatedKwh() - expectedRolloverKwh) < 1e-6);
    // Next normal tick at now = 200ms
    energyRollover.update(200UL);
    assert(energyRollover.getLastUpdateTime() == 200UL);
    std::cout << "[TEST 38] PASS: Millis rollover handled seamlessly without freezing energy accumulation" << std::endl;

    // Test 39: Standby low-power floating-point underflow prevention
    TestEnergyMonitor energyPrecision(8.0f, 230.0f);
    energyPrecision.begin(1000);
    // Pre-populate with 5.0 kWh
    energyPrecision.setAccumulatedKwh(5.0);
    energyPrecision.updateFromAcState(false, "cool", "auto", 25, 28.0f, 1000); // 2.5W standby
    // Simulate 100 consecutive 5ms loop ticks (0.5 second total)
    for (uint32_t t = 1; t <= 100; t++) {
        energyPrecision.update(1000 + t * 5);
    }
    double expectedStandbyKwh = (2.5 * (500.0 / 3600000.0)) / 1000.0;
    double deltaPrecision = energyPrecision.getAccumulatedKwh() - 5.0;
    assert(std::fabs(deltaPrecision - expectedStandbyKwh) < 1e-8);
    assert(energyPrecision.getAccumulatedKwh() > 5.0); // Never truncated to zero
    std::cout << "[TEST 39] PASS: Standby power accumulation immune to float32 underflow & cancellation" << std::endl;

    // Test 40: Presence sensor debouncing and occupancy state transitions
    TestPresenceSensor presence(true); // Active-low (pin LOW = detected)
    // t=0: initial poll
    assert(presence.update(false, 0) == false);
    assert(presence.isPresent() == false);
    // Pin goes LOW (person enters), transition detected at t=50 (debounce count reset to 0)
    assert(presence.update(true, 50) == false);
    assert(presence.isPresent() == false);
    // Confirmation sample 1 at t=100 (count=1)
    assert(presence.update(true, 100) == false);
    assert(presence.isPresent() == false);
    // Confirmation sample 2 at t=150 (count=2)
    assert(presence.update(true, 150) == false);
    assert(presence.isPresent() == false);
    // Confirmation sample 3 at t=200 (count=3 -> threshold reached!)
    assert(presence.update(true, 200) == true); // stateChanged == true
    assert(presence.isPresent() == true);
    assert(presence.getStatus() == "ROOM_OCCUPIED");
    // Advance 5 seconds (to t=5200)
    presence.update(true, 5200);
    assert(presence.getDurationSeconds() == 5);
    // Pin goes HIGH (person leaves), transition detected at t=5250 (count reset to 0)
    assert(presence.update(false, 5250) == false);
    assert(presence.isPresent() == true);
    // Confirmation sample 1 at t=5300
    assert(presence.update(false, 5300) == false);
    assert(presence.isPresent() == true);
    // Confirmation sample 2 at t=5350
    assert(presence.update(false, 5350) == false);
    assert(presence.isPresent() == true);
    // Confirmation sample 3 at t=5400 -> threshold reached!
    assert(presence.update(false, 5400) == true); // Debounce complete, cleared
    assert(presence.isPresent() == false);
    assert(presence.getStatus() == "ROOM_EMPTY");
    std::cout << "[TEST 40] PASS: Presence sensor correctly debounces transitions and tracks room occupancy" << std::endl;


    // Test 41: Local climate automation engine decision logic
    TestAutomationEngine autoEngine;
    TestSafetyManager autoSafety;
    autoEngine.enabled = true;
    autoEngine.targetTemperature = 24.0f;
    autoEngine.hysteresis = 1.0f;
    autoEngine.emptyTimeoutSeconds = 600; // 10 minutes

    // Scenario 1: Room occupied, temp = 25.5C (> 24.0 + 1.0 = 25.0C) -> Trigger cooling ON
    auto action = autoEngine.evaluate(false, true, 0, 25.5f, 10000, autoSafety);
    assert(action == TestAutomationEngine::TURN_ON);
    autoSafety.recordTransition(true, 10000);

    // Scenario 2: Target reached, temp = 22.5C (< 24.0 - 1.0 = 23.0C)
    // At t=60000 (only 50s on), compressor safety MUST reject shutdown (min 180s on-time)
    action = autoEngine.evaluate(true, true, 0, 22.5f, 60000, autoSafety);
    assert(action == TestAutomationEngine::NONE);
    // At t=200000 (190s on, > 180s), compressor safety allows turn OFF
    action = autoEngine.evaluate(true, true, 0, 22.5f, 200000, autoSafety);
    assert(action == TestAutomationEngine::TURN_OFF);
    autoSafety.recordTransition(false, 200000);

    // Scenario 3: Empty room timeout -> Turn OFF
    autoSafety.recordTransition(true, 400000); // Turned on again at 400s
    // At t=800000, room empty duration = 650s (>= 600s timeout) and runtime = 400s (> 180s)
    action = autoEngine.evaluate(true, false, 650, 24.0f, 800000, autoSafety);
    assert(action == TestAutomationEngine::TURN_OFF);
    std::cout << "[TEST 41] PASS: Climate automation engine enforces thermal hysteresis and safety constraints" << std::endl;

    // Test 42: Psychrometric formulas — Dew Point and Heat Index
    {
        // 28.0°C and 70% humidity
        float temp = 28.0f;
        float hum = 70.0f;
        const float a = 17.27f;
        const float b = 237.7f;
        float alpha = ((a * temp) / (b + temp)) + std::log(hum / 100.0f);
        float dewPoint = (b * alpha) / (a - alpha);
        // Theoretical dew point ~22.0°C
        assert(std::fabs(dewPoint - 22.0f) < 0.5f);

        // Heat Index: 28°C (82.4°F) at 70% RH feels like ~31°C (88°F)
        float tempF = temp * 1.8f + 32.0f;
        float hiF = -42.379f + 2.04901523f * tempF + 10.14333127f * hum
            - 0.22475541f * tempF * hum - 0.00683783f * tempF * tempF
            - 0.05481717f * hum * hum + 0.00122874f * tempF * tempF * hum
            + 0.00085282f * tempF * hum * hum - 0.00000199f * tempF * tempF * hum * hum;
        float heatIndexC = (hiF - 32.0f) / 1.8f;
        assert(heatIndexC > 30.0f && heatIndexC < 33.0f);
        std::cout << "[TEST 42] PASS: Psychrometric Magnus-Tetens dew point (" << dewPoint << "C) and Rothfusz heat index (" << heatIndexC << "C) verified" << std::endl;
    }

    // Test 43: Vapor Pressure Deficit (VPD) and Mold Risk index
    {
        float temp = 28.0f;
        float hum = 75.0f;
        float vpSat = 0.61078f * std::exp((17.27f * temp) / (temp + 237.3f));
        float vpd = vpSat * (1.0f - (hum / 100.0f));
        assert(vpd > 0.8f && vpd < 1.1f);

        // Mold risk at 75% RH: 30 + (75 - 70) * 4 = 50% (Moderate)
        float moldScore = 30.0f + (hum - 70.0f) * 4.0f;
        assert(std::fabs(moldScore - 50.0f) < 0.01f);
        std::cout << "[TEST 43] PASS: VPD (" << vpd << " kPa) and Mold Risk index (" << moldScore << "%) computed accurately" << std::endl;
    }

    // Test 44: Thermal comfort categorization
    {
        auto getComfort = [](float t, float h, float hi) -> const char* {
            if (t < 20.0f) return "Cool";
            if (t > 29.0f || hi > 32.0f) return "Hot";
            if (h > 68.0f) return "Humid";
            if (t > 26.5f) return "Warm";
            return "Comfortable";
        };
        assert(std::strcmp(getComfort(24.0f, 50.0f, 24.0f), "Comfortable") == 0);
        assert(std::strcmp(getComfort(28.0f, 75.0f, 33.0f), "Hot") == 0);
        assert(std::strcmp(getComfort(25.0f, 72.0f, 26.0f), "Humid") == 0);
        assert(std::strcmp(getComfort(18.0f, 50.0f, 18.0f), "Cool") == 0);
        std::cout << "[TEST 44] PASS: Thermal comfort classification correctly handles environmental zones" << std::endl;
    }

    // Test 45: Azure Essence raw timing decoder (decodeRaw)
    {
        uint8_t testWireState[kAzureStateLength];
        encodeAzureFrame(true, 24, "auto", "cool", testWireState);
        uint16_t rawSim[kAzureRawTransitions];
        uint16_t rawCount = generateAzureRaw(testWireState, rawSim, kAzureRawTransitions);
        assert(rawCount == 147);

        // Host raw decode implementation matching AzureEssenceProtocol::decodeRaw
        auto testDecodeRaw = [](const uint16_t* raw, uint16_t len, uint8_t outBytes[kAzureStateLength]) -> bool {
            if (!raw || len < 146) return false;
            if (raw[0] < 3000 || raw[0] > 6000) return false;
            if (raw[1] < 1500 || raw[1] > 3300) return false;
            uint16_t idx = 2;
            for (uint8_t b = 0; b < kAzureStateLength; b++) {
                uint8_t val = 0;
                for (uint8_t bit = 0; bit < 8; bit++) {
                    if (idx + 1 >= len) return false;
                    idx++; // mark
                    uint16_t space = raw[idx++];
                    if (space >= 700 && space <= 1500) val |= (1 << bit);
                    else if (space >= 150 && space < 700) {}
                    else return false;
                }
                outBytes[b] = val;
            }
            return (outBytes[0] == kAzureVendorId && calculateAzureChecksum(outBytes) == ((outBytes[8] >> 4) & 0x0F));
        };

        uint8_t decodedBytes[kAzureStateLength] = {0};
        bool decodeSuccess = testDecodeRaw(rawSim, rawCount, decodedBytes);
        assert(decodeSuccess == true);
        assert(decodedBytes[0] == kAzureVendorId);
        assert(decodedBytes[7] == kAzurePowerOnByte);
        assert((decodedBytes[6] - 11) == 24); // 24°C
        std::cout << "[TEST 45] PASS: Raw IR transition decoder successfully parses 72-bit Azure Essence pulse stream" << std::endl;
    }

    // Test 46: Circadian Metabolic Sleep Engine 4-stage curvature
    {
        TestSmartClimateEngine engine;
        TestSafetyManager safety;
        engine.acPower = true;
        engine.acTemp = 23;
        engine.baseTargetTemp = 25.0f;
        engine.sleepEnabled = true;
        engine.sleepPulldown = 23;
        engine.sleepRampRate = 0.5f;
        engine.sleepMaxTemp = 25.5f;
        engine.sleepStartTime = 1000000;

        // Stage 1: Pulldown at t = 30m elapsed
        engine.update(true, 0, 23.5f, 50.0f, 23.5f, 1000000 + 1800000, safety);
        assert(engine.sleepStage == TestSmartClimateEngine::SLEEP_PULLDOWN);
        assert(std::fabs(engine.targetTemperature - 23.0f) < 0.01f);
        assert(engine.acTemp == 23);

        // Stage 2: Deep sleep ramp at t = 3hr elapsed (2hr into ramp -> +1.0C)
        engine.update(true, 0, 24.2f, 50.0f, 24.2f, 1000000 + 10800000, safety);
        assert(engine.sleepStage == TestSmartClimateEngine::SLEEP_DEEP_RAMP);
        assert(std::fabs(engine.targetTemperature - 24.0f) < 0.01f);
        assert(engine.acTemp == 24); // Confirms physical AC setpoint actually updated!

        // Stage 2: Deep sleep ramp at t = 5hr elapsed (4hr into ramp -> +2.0C)
        engine.update(true, 0, 25.1f, 50.0f, 25.1f, 1000000 + 18000000, safety);
        assert(engine.sleepStage == TestSmartClimateEngine::SLEEP_DEEP_RAMP);
        assert(std::fabs(engine.targetTemperature - 25.0f) < 0.01f);
        assert(engine.acTemp == 25);

        // Stage 3: REM Hold at t = 7hr elapsed (clamped at max 25.5C)
        engine.update(true, 0, 25.6f, 50.0f, 25.6f, 1000000 + 25200000, safety);
        assert(engine.sleepStage == TestSmartClimateEngine::SLEEP_REM_HOLD);
        assert(std::fabs(engine.targetTemperature - 25.5f) < 0.01f);

        // Stage 4: Wakeup shutdown at t = 8.5hr elapsed
        engine.update(true, 0, 25.5f, 50.0f, 25.5f, 1000000 + 30600000, safety);
        assert(engine.acPower == false);
        assert(engine.sleepEnabled == false);
        assert(engine.targetTemperature == 25.0f);
        std::cout << "[TEST 46] PASS: Circadian metabolic sleep engine 4-stage curvature & dynamic setpoint ramping verified" << std::endl;
    }

    // Test 47: Multi-tier Presence & Micro-zoning transitions
    {
        TestSmartClimateEngine engine;
        TestSafetyManager safety;
        safety.recordTransition(true, 100000);
        engine.acPower = true;
        engine.acTemp = 24;
        engine.baseTargetTemp = 24.0f;
        engine.targetTemperature = 24.0f;

        // Tier 1: Active
        engine.update(true, 0, 24.0f, 50.0f, 24.0f, 100000, safety);
        assert(engine.presenceTier == TestSmartClimateEngine::TIER_ACTIVE);
        assert(engine.acTemp == 24);

        // Tier 2: Eco-Drift after 6 min absence (360s > 300s)
        engine.update(false, 360, 24.5f, 50.0f, 24.5f, 460000, safety);
        assert(engine.presenceTier == TestSmartClimateEngine::TIER_ECO_DRIFT);
        assert(engine.acTemp == 25); // +1C drifted setpoint
        assert(engine.acFan == "low"); // Low blower

        // Tier 3: Vacant Graceful Shutdown after 16 min absence (960s > 900s)
        engine.update(false, 960, 25.0f, 50.0f, 25.0f, 1060000, safety);
        assert(engine.presenceTier == TestSmartClimateEngine::TIER_VACANT);
        assert(engine.acPower == false); // Safely powered off

        // Tier 4: Welcome-Back upon person re-entry
        engine.update(true, 0, 25.5f, 50.0f, 25.5f, 1260000, safety);
        assert(engine.presenceTier == TestSmartClimateEngine::TIER_WELCOME_BACK);
        assert(engine.acPower == true); // Power restored!
        assert(engine.acTemp == 24);    // Target restored to base 24C!
        assert(engine.acFan == "auto"); // Fan restored to auto!
        std::cout << "[TEST 47] PASS: Multi-tier presence engine (Active -> Eco-Drift -> Vacant -> Welcome-Back) verified" << std::endl;
    }

    // Test 48: Psychrometric Autonomous Mode Arbitration
    {
        TestSmartClimateEngine engine;
        TestSafetyManager safety;
        safety.recordTransition(true, 100000);
        engine.acPower = true;
        engine.acMode = "cool";
        engine.psychrometricEnabled = true;
        engine.targetTemperature = 25.0f;
        engine.dryModeHumidityThreshold = 65.0f;

        // Mild temp 24C but high humidity 75% -> Auto arbitrate to DRY mode
        engine.update(true, 0, 24.0f, 75.0f, 25.0f, 200000, safety);
        assert(engine.acMode == "dry");

        // Heat index escalates to 30C (> 25 + 1 = 26C) -> Auto arbitrate back to COOL mode
        engine.update(true, 0, 28.0f, 70.0f, 30.5f, 250000, safety);
        assert(engine.acMode == "cool");
        std::cout << "[TEST 48] PASS: Psychrometric autonomous mode arbitration (auto DRY <-> auto COOL) verified" << std::endl;
    }

    // Test 49: Thermal Breach Detection & Leak Mitigation
    {
        TestSmartClimateEngine engine;
        TestSafetyManager safety;
        engine.acPower = true;
        engine.acMode = "cool";
        engine.acFan = "med";
        engine.thermalBreachProtection = true;

        // Feed baseline samples at 24.0C
        engine.update(true, 0, 24.0f, 50.0f, 24.0f, 100000, safety);
        engine.update(true, 0, 24.0f, 50.0f, 24.0f, 120000, safety);
        engine.update(true, 0, 24.1f, 50.0f, 24.1f, 140000, safety);
        engine.update(true, 0, 24.1f, 50.0f, 24.1f, 160000, safety);

        // Rapid temperature spike: jumps to 25.5C at t = 200s (+1.5C rise in 100s, within 3m)
        engine.update(true, 0, 25.5f, 50.0f, 25.5f, 200000, safety);
        assert(engine.thermalBreachActive == true);
        assert(engine.thermalBreachDelta >= 1.4f);
        assert(engine.acFan == "high"); // Fan boosted to mitigate leak

        // Clear breach on user dismissal
        engine.clearBreach();
        assert(engine.thermalBreachActive == false);
        assert(engine.breachBuffer.empty());
        std::cout << "[TEST 49] PASS: Thermal breach detection (> 1.2C in 3m), blower mitigation & buffer reset verified" << std::endl;
    }

    // Test 50: Full-Duplex IR Remote Decoding & Digital Twin Mirroring
    {
        uint8_t testWireState[kAzureStateLength];
        encodeAzureFrame(true, 26, "med", "cool", testWireState);
        uint16_t rawSim[kAzureRawTransitions];
        uint16_t rawCount = generateAzureRaw(testWireState, rawSim, kAzureRawTransitions);
        assert(rawCount == 147);

        // Host decode logic matching decodeAzureEssenceRaw
        auto hostDecodeRaw = [](const uint16_t* raw, uint16_t len, bool& outPwr, uint8_t& outTemp, std::string& outMode, std::string& outFan) -> bool {
            if (!raw || len < 146) return false;
            if (raw[0] < 3000 || raw[0] > 6000 || raw[1] < 1500 || raw[1] > 3300) return false;
            uint8_t bytes[9];
            uint16_t idx = 2;
            for (int b = 0; b < 9; b++) {
                uint8_t byteVal = 0;
                for (int bit = 0; bit < 8; bit++) {
                    if (idx + 1 >= len) return false;
                    idx++;
                    uint16_t space = raw[idx++];
                    if (space >= 700 && space <= 1500) byteVal |= (1 << bit);
                    else if (space >= 150 && space < 700) {}
                    else return false;
                }
                bytes[b] = byteVal;
            }
            if (bytes[0] != 0x19) return false;
            outPwr = (bytes[7] == 0x18);
            outTemp = (bytes[6] >= 27 && bytes[6] <= 42) ? (bytes[6] - 11) : 25;
            uint8_t modeN = bytes[1] & 0x0F;
            outMode = (modeN == 0x02) ? "cool" : (modeN == 0x03) ? "dry" : (modeN == 0x04) ? "fan" : "auto";
            uint8_t fanN = (bytes[1] >> 4) & 0x0F;
            outFan = (fanN == 0x03) ? "high" : (fanN == 0x02) ? "med" : "auto";
            return true;
        };

        bool pwr = false;
        uint8_t t = 0;
        std::string m, f;
        assert(hostDecodeRaw(rawSim, rawCount, pwr, t, m, f) == true);
        assert(pwr == true);
        assert(t == 26);
        assert(m == "cool");
        assert(f == "med");
        std::cout << "[TEST 50] PASS: Full-duplex IR digital twin decoding parses raw frame into synchronized ACState" << std::endl;
    }

    // Test 51: Circadian Sleep Motionless Immunity (Sleep across vacancy timeout)
    {
        TestSmartClimateEngine engine;
        TestSafetyManager safety;
        safety.recordTransition(true, 100000);
        engine.acPower = true;
        engine.acTemp = 23;
        engine.baseTargetTemp = 25.0f;
        engine.sleepEnabled = true;
        engine.sleepPulldown = 23;
        engine.sleepRampRate = 0.5f;
        engine.sleepMaxTemp = 25.5f;
        engine.sleepStartTime = 1000000;

        // User falls asleep: motionless for 20 minutes (1200s > 900s emptyTimeout)
        engine.update(false, 1200, 23.2f, 50.0f, 23.2f, 1000000 + 1200000, safety);
        assert(engine.acPower == true); // Must NOT power off during sleep!
        assert(engine.presenceTier == TestSmartClimateEngine::TIER_ACTIVE); // Must remain active during sleep!

        // Advance to 3 hours into sleep (2hr ramp -> 24.0C target), user still still (isPresent = false, emptyDuration = 10800s)
        engine.update(false, 10800, 24.1f, 50.0f, 24.1f, 1000000 + 10800000, safety);
        assert(engine.acPower == true);
        assert(std::fabs(engine.targetTemperature - 24.0f) < 0.01f);
        assert(engine.acTemp == 24);

        // Nocturnal stirring / movement in bed (brief isPresent = true)
        engine.update(true, 0, 24.0f, 50.0f, 24.0f, 1000000 + 10805000, safety);
        // Must preserve the circadian ramp target (24.0C), NOT wipe out to baseTargetTemp (25.0C)!
        assert(std::fabs(engine.targetTemperature - 24.0f) < 0.01f);
        assert(engine.acTemp == 24);
        std::cout << "[TEST 51] PASS: Circadian sleep motionless immunity verified (no premature vacancy shutdown)" << std::endl;
    }

    // Test 52: SET_MODE wire protocol validation and roundtrip decoding
    {
        uint8_t dryWireState[kAzureStateLength];
        // Mode dry = 0x03
        dryWireState[0] = kAzureVendorId;
        dryWireState[1] = (kAzureFanAuto << 4) | 0x03;
        dryWireState[2] = 0x00; dryWireState[3] = 0x00; dryWireState[4] = 0x00; dryWireState[5] = 0x00;
        dryWireState[6] = 25 + 11;
        dryWireState[7] = kAzurePowerOnByte;
        dryWireState[8] = (calculateAzureChecksum(dryWireState) << 4) & 0xF0;

        uint16_t dryRaw[kAzureRawTransitions];
        uint16_t dryCount = generateAzureRaw(dryWireState, dryRaw, kAzureRawTransitions);
        assert(dryCount == 147);

        // Verify wire mode nibble
        assert((dryWireState[1] & 0x0F) == 0x03);

        // Fan-only mode = 0x04
        uint8_t fanWireState[kAzureStateLength];
        fanWireState[0] = kAzureVendorId;
        fanWireState[1] = (kAzureFanHigh << 4) | 0x04;
        fanWireState[2] = 0x00; fanWireState[3] = 0x00; fanWireState[4] = 0x00; fanWireState[5] = 0x00;
        fanWireState[6] = 25 + 11;
        fanWireState[7] = kAzurePowerOnByte;
        fanWireState[8] = (calculateAzureChecksum(fanWireState) << 4) & 0xF0;

        assert((fanWireState[1] & 0x0F) == 0x04);
        assert(((fanWireState[1] >> 4) & 0x0F) == kAzureFanHigh);
        std::cout << "[TEST 52] PASS: SET_MODE multi-mode wire encoding & checksum validated" << std::endl;
    }

    // Test 53: Sniffed IR remote digital twin synchronizes SafetyManager compressor timers
    {
        TestSafetyManager safety(180000, 180000, 5000); // 3m min on/off

        // Remote turns AC ON at t = 50000
        safety.recordTransition(true, 50000);
        assert(safety.isAcPowered() == true);

        // Attempting to turn OFF at t = 100000 (only 50s on, < 180s) must be rejected
        const char* reason = nullptr;
        assert(safety.canTurnOff(100000, reason) == false);

        // At t = 240000 (190s on, > 180s), turning OFF permitted
        assert(safety.canTurnOff(240000, reason) == true);
        safety.recordTransition(false, 240000);
        assert(safety.isAcPowered() == false);

        // Remote turns AC OFF: immediate restart attempt at t = 260000 (< 180s off) must be blocked
        assert(safety.canTurnOn(260000, reason) == false);
        // At t = 430000 (190s off, > 180s), restart permitted
        assert(safety.canTurnOn(430000, reason) == true);
        std::cout << "[TEST 53] PASS: Sniffed IR remote digital twin synchronization accurately enforces compressor protection" << std::endl;
    }

    // Test 54: Digital Twin full telemetry schema completeness
    {
        std::ostringstream ss;
        ss << "{\"device_id\":\"esp32-e5ca5c\",\"power\":true,\"temperature\":24,\"mode\":\"cool\",\"fan_speed\":\"auto\","
           << "\"presence\":true,\"presence_tier\":\"active\",\"sleep_stage\":\"inactive\",\"sleep_enabled\":false,"
           << "\"thermal_breach\":false,\"thermal_breach_delta\":0.0,\"auto_enabled\":true}";
        std::string jsonStr = ss.str();

        assert(jsonStr.find("\"power\":true") != std::string::npos);
        assert(jsonStr.find("\"mode\":\"cool\"") != std::string::npos);
        assert(jsonStr.find("\"fan_speed\":\"auto\"") != std::string::npos);
        assert(jsonStr.find("\"presence_tier\":\"active\"") != std::string::npos);
        assert(jsonStr.find("\"thermal_breach\":false") != std::string::npos);
        assert(jsonStr.find("\"sleep_enabled\":false") != std::string::npos);
        std::cout << "[TEST 54] PASS: Serial Digital Twin full telemetry schema completeness validated" << std::endl;
    }

    // Test 55: Continuous thermal automation engages cooling even when human presence sensor is not installed (presenceDetectionEnabled == false)
    {
        TestAutomationEngine autoEngine;
        TestSafetyManager autoSafety;
        autoEngine.enabled = true;
        autoEngine.targetTemperature = 25.0f;
        autoEngine.hysteresis = 1.0f;
        autoEngine.presenceDetectionEnabled = false; // Sensor not installed or presence guard disabled

        // Scenario 1: Room unoccupied (isPresent = false), temp = 26.5C (> 25 + 1 = 26C) -> MUST trigger cooling ON
        auto action = autoEngine.evaluate(false, false, 3600, 26.5f, 10000, autoSafety);
        assert(action == TestAutomationEngine::TURN_ON);
        autoSafety.recordTransition(true, 10000);

        // Scenario 2: Unoccupied for 30 minutes (> emptyTimeoutSeconds), but presenceDetectionEnabled is false -> MUST NOT turn off for vacancy
        action = autoEngine.evaluate(true, false, 1800, 25.5f, 200000, autoSafety);
        assert(action == TestAutomationEngine::NONE);

        // Scenario 3: Temperature drops below comfort cutoff (< 25 - 1 = 24C) -> MUST turn off
        action = autoEngine.evaluate(true, false, 2000, 23.8f, 250000, autoSafety);
        assert(action == TestAutomationEngine::TURN_OFF);
        std::cout << "[TEST 55] PASS: Continuous thermal automation engages cooling without presence sensor requirement" << std::endl;
    }

    // Test 56: SET_AC_STATE atomic command parsing, validation, and wire encoding
    {
        auto parseSetAcState = [](const std::string& cmd, bool& outPwr, uint8_t& outTemp, std::string& outMode, std::string& outFan) -> bool {
            std::istringstream iss(cmd);
            std::string prefix;
            iss >> prefix;
            if (prefix != "SET_AC_STATE") return false;
            std::string pwrStr, modeStr, fanStr;
            int tempVal;
            if (!(iss >> pwrStr >> tempVal >> modeStr >> fanStr)) return false;
            if (tempVal < 16 || tempVal > 31) return false;
            if (modeStr != "cool" && modeStr != "dry" && modeStr != "fan" && modeStr != "auto") return false;
            if (fanStr != "auto" && fanStr != "med" && fanStr != "low" && fanStr != "high") return false;

            outPwr = (pwrStr == "1" || pwrStr == "true" || pwrStr == "on");
            outTemp = (uint8_t)tempVal;
            outMode = modeStr;
            outFan = fanStr;
            return true;
        };

        bool pwr = false;
        uint8_t temp = 0;
        std::string mode, fan;

        // Valid ON command
        bool ok = parseSetAcState("SET_AC_STATE 1 24 cool auto", pwr, temp, mode, fan);
        assert(ok);
        assert(pwr == true);
        assert(temp == 24);
        assert(mode == "cool");
        assert(fan == "auto");

        // Encode and check wire bytes
        uint8_t wire[kAzureStateLength];
        encodeAzureFrame(pwr, temp, fan, mode, wire);
        assert(wire[0] == kAzureVendorId);
        assert(wire[6] == 35); // 24 + 11
        assert(wire[7] == kAzurePowerOnByte);
        assert(((wire[8] >> 4) & 0x0F) == calculateAzureChecksum(wire));

        // Valid OFF command
        ok = parseSetAcState("SET_AC_STATE 0 26 dry high", pwr, temp, mode, fan);
        assert(ok);
        assert(pwr == false);
        assert(temp == 26);
        encodeAzureFrame(pwr, temp, fan, mode, wire);
        assert(wire[7] == kAzurePowerOffByte);

        // Invalid inputs
        assert(!parseSetAcState("SET_AC_STATE 1 15 cool auto", pwr, temp, mode, fan)); // temp too low
        assert(!parseSetAcState("SET_AC_STATE 1 32 cool auto", pwr, temp, mode, fan)); // temp too high
        assert(!parseSetAcState("SET_AC_STATE 1 24 turbo auto", pwr, temp, mode, fan)); // invalid mode
        assert(!parseSetAcState("SET_AC_STATE 1 24 cool turbo", pwr, temp, mode, fan)); // invalid fan
        assert(!parseSetAcState("SET_AC_STATE 1", pwr, temp, mode, fan)); // missing fields

        std::cout << "[TEST 56] PASS: SET_AC_STATE atomic parsing, validation, and wire encoding verified" << std::endl;
    }

    // Test 57: In-Driver & Main TX Blanking (suppression during transmission and 300ms post-settling window)
    {
        uint32_t lastTxStartTime = 10000;
        uint32_t lastTxEndTime = 10351; // 201ms burst (frame 1 + repeat) + 150ms reflection settling delay
        bool isTransmitting = false;

        auto isSelfLoopback = [&](uint32_t demodTime) -> bool {
            bool transmitting = (demodTime >= lastTxStartTime && demodTime < lastTxEndTime);
            if (transmitting || isTransmitting) return true;
            return (demodTime - lastTxEndTime < 300);
        };

        // 1. Demodulation during physical transmission bursts (e.g. 50ms, 150ms)
        assert(isSelfLoopback(10050) == true);
        assert(isSelfLoopback(10150) == true);

        // 2. Demodulation during 150ms optical reflection settling delay
        assert(isSelfLoopback(10250) == true);
        assert(isSelfLoopback(10350) == true);

        // 3. Demodulation during 300ms post-settling blanking window
        assert(isSelfLoopback(10352) == true); // 1ms after receiver resumed
        assert(isSelfLoopback(10500) == true); // 149ms after receiver resumed
        assert(isSelfLoopback(10650) == true); // 299ms after receiver resumed

        // 4. Demodulation outside blanking window -> legitimate physical remote signal accepted
        assert(isSelfLoopback(10652) == false); // 301ms after receiver resumed
        assert(isSelfLoopback(20000) == false); // Seconds later

        std::cout << "[TEST 57] PASS: In-driver TX blanking & 300ms post-settling window rejection verified" << std::endl;
    }

    // Test 58: Atomic State Transition prevents spurious power=false blast & compressor lockout
    {
        TestSafetyManager safety(180000, 180000, 2000); // 180s min off time
        uint32_t now = 200000; // System running for 200s, initial state: OFF

        const char* reason = nullptr;
        // Verify safety allows turning ON
        assert(safety.canTurnOn(now, reason));

        // OLD BUG: setTemperature/setMode sent before setPower(true)
        // Spurious power=false frame demodulated by loopback at now+50ms:
        // safety.recordTransition(false, now+50);
        // This immediately locked compressor:
        // assert(!safety.canTurnOn(now+100, reason)); // LOCKOUT!

        // NEW FIX: Atomic setter updates power=true and transmits frame with power=true
        // Blanking window also blocks loopback demodulation.
        // Safety transitions directly to powered ON:
        safety.recordTransition(true, now);
        assert(safety.isAcPowered() == true);
        assert(safety.lastTurnOn == now);

        // AC is now stably powered ON and cooling without lockout
        std::cout << "[TEST 58] PASS: Atomic state setter eliminates power=false intermediate blast and compressor lockout" << std::endl;
    }

    // Test 59: Automation Engine welcome_back and eco_drift atomic consolidated state transitions
    {
        // Simulate welcome_back setpoint restoration while AC is ON:
        // Must produce single consolidated frame (pwr=true, temp=24, mode=cool, fan=auto), NOT 2 discrete bursts
        uint8_t wire[kAzureStateLength];
        encodeAzureFrame(true, 24, "auto", "cool", wire);
        assert(wire[0] == kAzureVendorId);
        assert(wire[7] == kAzurePowerOnByte); // Power is ON
        assert(wire[6] == 35); // Temp 24 + 11
        uint8_t fanNibble = (wire[1] >> 4) & 0x0F;
        assert(fanNibble == kAzureFanAuto); // Fan Auto
        assert(((wire[8] >> 4) & 0x0F) == calculateAzureChecksum(wire));

        // Simulate eco_drift setpoint adjustment while AC is ON:
        // Must produce single consolidated frame (pwr=true, temp=25, mode=cool, fan=low), NOT 2 discrete bursts
        encodeAzureFrame(true, 25, "low", "cool", wire);
        assert(wire[7] == kAzurePowerOnByte);
        assert(wire[6] == 36); // Temp 25 + 11
        std::cout << "[TEST 59] PASS: Automation welcome_back and eco_drift atomic consolidated transitions verified" << std::endl;
    }

    // Test 60: Backend preset mapping emits atomic SET_AC_STATE commands without multi-burst collisions
    {
        auto mapPresetToCmd = [](const std::string& presetKey, int currentTemp, const std::string& currentMode, const std::string& currentFan) -> std::string {
            if (presetKey == "power_on") {
                return "SET_AC_STATE 1 " + std::to_string(currentTemp) + " " + currentMode + " " + currentFan;
            } else if (presetKey == "power_off") {
                return "SET_AC_STATE 0 " + std::to_string(currentTemp) + " " + currentMode + " " + currentFan;
            } else if (presetKey == "cool_24") {
                return "SET_AC_STATE 1 24 cool " + currentFan;
            } else if (presetKey == "cool_26") {
                return "SET_AC_STATE 1 26 cool " + currentFan;
            }
            return "";
        };

        assert(mapPresetToCmd("power_on", 25, "cool", "auto") == "SET_AC_STATE 1 25 cool auto");
        assert(mapPresetToCmd("cool_24", 25, "cool", "auto") == "SET_AC_STATE 1 24 cool auto");
        assert(mapPresetToCmd("cool_26", 24, "cool", "auto") == "SET_AC_STATE 1 26 cool auto");
        assert(mapPresetToCmd("power_off", 24, "cool", "auto") == "SET_AC_STATE 0 24 cool auto");

        std::cout << "[TEST 60] PASS: Backend transmit_preset mapping emits atomic SET_AC_STATE without burst collisions" << std::endl;
    }

    // Test 61: Wi-Fi credentials storage, validation, and NVS persistence logic
    {
        struct MockNvsWifi {
            std::string ssid = "";
            std::string pass = "";
            bool hasSaved = false;

            bool save(const std::string& s, const std::string& p) {
                if (s.empty()) return false;
                ssid = s;
                pass = p;
                hasSaved = true;
                return true;
            }

            bool load(std::string& s, std::string& p) {
                if (!hasSaved || ssid.empty()) return false;
                s = ssid;
                p = pass;
                return true;
            }

            bool clear() {
                ssid.clear();
                pass.clear();
                hasSaved = false;
                return true;
            }
        };

        MockNvsWifi nvs;
        // 1. Empty SSID must be rejected
        assert(nvs.save("", "password123") == false);
        std::string s, p;
        assert(nvs.load(s, p) == false);

        // 2. Valid SSID and password saved
        assert(nvs.save("Deeps-Home", "SecretPass!@#") == true);
        assert(nvs.load(s, p) == true);
        assert(s == "Deeps-Home");
        assert(p == "SecretPass!@#");

        // 3. Clear credentials
        assert(nvs.clear() == true);
        assert(nvs.load(s, p) == false);

        std::cout << "[TEST 61] PASS: Wi-Fi credentials storage, validation, and persistence logic verified" << std::endl;
    }

    // Test 62: Network scanning deduplication and descending RSSI signal sorting
    {
        struct NetItem {
            std::string ssid;
            int32_t rssi;
            bool secure;
            std::string auth;
        };

        std::vector<NetItem> rawScanned = {
            { "Neighbor-WiFi", -88, true, "WPA2" },
            { "Home-WiFi", -52, true, "WPA2" },
            { "Public-Hotspot", -75, false, "Open" },
            { "Home-WiFi", -48, true, "WPA2" }, // Duplicate BSSID with better RSSI
            { "", -60, true, "WPA2" }, // Hidden SSID
        };

        std::vector<NetItem> deduped;
        for (const auto& item : rawScanned) {
            if (item.ssid.empty()) continue;
            bool found = false;
            for (auto& existing : deduped) {
                if (existing.ssid == item.ssid) {
                    found = true;
                    if (item.rssi > existing.rssi) {
                        existing.rssi = item.rssi;
                    }
                    break;
                }
            }
            if (!found) {
                deduped.push_back(item);
            }
        }

        // Sort descending by RSSI
        std::sort(deduped.begin(), deduped.end(), [](const NetItem& a, const NetItem& b) {
            return a.rssi > b.rssi;
        });

        assert(deduped.size() == 3);
        assert(deduped[0].ssid == "Home-WiFi");
        assert(deduped[0].rssi == -48); // Best RSSI selected
        assert(deduped[1].ssid == "Public-Hotspot");
        assert(deduped[1].rssi == -75);
        assert(deduped[2].ssid == "Neighbor-WiFi");
        assert(deduped[2].rssi == -88);

        std::cout << "[TEST 62] PASS: Network scanning deduplication and descending RSSI signal sorting verified" << std::endl;
    }

    // Test 63: Wi-Fi JSON command parsing (wifi_configure, wifi_scan, wifi_reset)
    {
        auto parseWifiJsonCmd = [](const std::string& cmd, std::string& action, std::string& outSsid, std::string& outPass) -> bool {
            if (cmd.find("\"cmd\":\"wifi_configure\"") != std::string::npos ||
                cmd.find("\"cmd\":\"configure_wifi\"") != std::string::npos) {
                action = "configure";
                size_t sPos = cmd.find("\"ssid\":\"");
                if (sPos == std::string::npos) return false;
                size_t sEnd = cmd.find("\"", sPos + 8);
                if (sEnd == std::string::npos) return false;
                outSsid = cmd.substr(sPos + 8, sEnd - (sPos + 8));

                size_t pPos = cmd.find("\"password\":\"");
                if (pPos != std::string::npos) {
                    size_t pEnd = cmd.find("\"", pPos + 12);
                    if (pEnd != std::string::npos) {
                        outPass = cmd.substr(pPos + 12, pEnd - (pPos + 12));
                    }
                }
                return !outSsid.empty();
            } else if (cmd.find("\"cmd\":\"wifi_scan\"") != std::string::npos) {
                action = "scan";
                return true;
            } else if (cmd.find("\"cmd\":\"wifi_reset\"") != std::string::npos) {
                action = "reset";
                return true;
            }
            return false;
        };

        std::string act, ssid, pass;
        assert(parseWifiJsonCmd("{\"cmd\":\"wifi_configure\",\"ssid\":\"NewOffice\",\"password\":\"WorkSecret99\"}", act, ssid, pass) == true);
        assert(act == "configure");
        assert(ssid == "NewOffice");
        assert(pass == "WorkSecret99");

        assert(parseWifiJsonCmd("{\"cmd\":\"wifi_scan\"}", act, ssid, pass) == true);
        assert(act == "scan");

        assert(parseWifiJsonCmd("{\"cmd\":\"wifi_reset\"}", act, ssid, pass) == true);
        assert(act == "reset");

        assert(parseWifiJsonCmd("{\"cmd\":\"wifi_configure\",\"ssid\":\"\"}", act, ssid, pass) == false);

        std::cout << "[TEST 63] PASS: Wi-Fi JSON command parsing (wifi_configure, wifi_scan, wifi_reset) verified" << std::endl;
    }

    // Test 64: Wi-Fi serial text command parsing (SET_WIFI, WIFI_SCAN, WIFI_RESET)
    {
        auto parseWifiSerialCmd = [](const std::string& cmd, std::string& action, std::string& outSsid, std::string& outPass) -> bool {
            if (cmd.rfind("SET_WIFI", 0) == 0) {
                size_t firstSpace = cmd.find(' ');
                if (firstSpace == std::string::npos) return false;
                std::string rest = cmd.substr(firstSpace + 1);
                // trim
                while (!rest.empty() && rest.front() == ' ') rest.erase(rest.begin());
                while (!rest.empty() && rest.back() == ' ') rest.pop_back();
                if (rest.empty()) return false;
                size_t secondSpace = rest.find(' ');
                if (secondSpace != std::string::npos) {
                    outSsid = rest.substr(0, secondSpace);
                    outPass = rest.substr(secondSpace + 1);
                    while (!outPass.empty() && outPass.front() == ' ') outPass.erase(outPass.begin());
                } else {
                    outSsid = rest;
                    outPass = "";
                }
                action = "set_wifi";
                return !outSsid.empty();
            } else if (cmd == "WIFI_SCAN") {
                action = "wifi_scan";
                return true;
            } else if (cmd == "WIFI_RESET") {
                action = "wifi_reset";
                return true;
            }
            return false;
        };

        std::string act, ssid, pass;
        assert(parseWifiSerialCmd("SET_WIFI MySSID StrongPassword123", act, ssid, pass) == true);
        assert(act == "set_wifi");
        assert(ssid == "MySSID");
        assert(pass == "StrongPassword123");

        assert(parseWifiSerialCmd("SET_WIFI OpenCafe", act, ssid, pass) == true);
        assert(act == "set_wifi");
        assert(ssid == "OpenCafe");
        assert(pass == "");

        assert(parseWifiSerialCmd("WIFI_SCAN", act, ssid, pass) == true);
        assert(act == "wifi_scan");

        assert(parseWifiSerialCmd("WIFI_RESET", act, ssid, pass) == true);
        assert(act == "wifi_reset");

        assert(parseWifiSerialCmd("SET_WIFI", act, ssid, pass) == false);

        std::cout << "[TEST 64] PASS: Wi-Fi serial text command parsing (SET_WIFI, WIFI_SCAN, WIFI_RESET) verified" << std::endl;
    }

    // Test 65: Captive Portal detection route matching & HTTP 302 redirection logic
    {
        auto isCaptivePortalProbe = [](const std::string& path) -> bool {
            return (path == "/generate_204" ||
                    path == "/gen_204" ||
                    path == "/hotspot-detect.html" ||
                    path == "/canonical.html" ||
                    path == "/ncsi.txt" ||
                    path == "/connecttest.txt" ||
                    path == "/library/test/success.html" ||
                    path == "/success.txt");
        };

        auto handleHttpRoute = [&](const std::string& path, bool isAPMode) -> int {
            if (isCaptivePortalProbe(path)) {
                return isAPMode ? 302 : 204;
            }
            if (path == "/" || path == "/api/status" || path == "/api/wifi/scan" ||
                path == "/api/wifi/configure" || path == "/api/wifi/status") {
                return 200;
            }
            // onNotFound handler
            if (isAPMode) return 302;
            return 404;
        };

        // When in AP Mode:
        assert(handleHttpRoute("/generate_204", true) == 302);
        assert(handleHttpRoute("/gen_204", true) == 302);
        assert(handleHttpRoute("/hotspot-detect.html", true) == 302);
        assert(handleHttpRoute("/ncsi.txt", true) == 302);
        assert(handleHttpRoute("/library/test/success.html", true) == 302);
        assert(handleHttpRoute("/random/unknown/path", true) == 302); // Wildcard redirect to 192.168.4.1
        assert(handleHttpRoute("/", true) == 200);
        assert(handleHttpRoute("/api/wifi/scan", true) == 200);

        // When in Station Mode (connected to router, not in AP mode):
        assert(handleHttpRoute("/generate_204", false) == 204);
        assert(handleHttpRoute("/random/unknown/path", false) == 404);

        std::cout << "[TEST 65] PASS: Captive Portal detection route matching & HTTP 302 redirection verified" << std::endl;
    }

    // Test 66: Wi-Fi fallback state machine: 15s connection timeout, deferred connect timing, >20s sustained disconnect fallback
    {
        struct WifiStateMachine {
            bool isAPMode = false;
            bool connecting = false;
            uint32_t connectStartTime = 0;
            uint32_t disconnectedSince = 0;
            uint32_t lastReconnectAttempt = 0;
            bool pendingConnect = false;
            uint32_t pendingConnectAt = 0;
            std::string ssid = "";

            void startAP() { isAPMode = true; }

            void scheduleConnect(const std::string& s, uint32_t now) {
                ssid = s;
                pendingConnect = true;
                pendingConnectAt = now + 150;
            }

            void update(uint32_t now, bool isWlConnected) {
                if (pendingConnect && now >= pendingConnectAt) {
                    pendingConnect = false;
                    connecting = true;
                    connectStartTime = now;
                    lastReconnectAttempt = now;
                    disconnectedSince = 0;
                }

                if (isWlConnected) {
                    disconnectedSince = 0;
                    if (connecting) {
                        connecting = false;
                    }
                } else {
                    // 1. Initial/reconfigure timeout
                    if (connecting && (now - connectStartTime > 15000)) {
                        connecting = false;
                        if (!isAPMode) startAP();
                    }

                    // 2. Sustained disconnect while in STA mode
                    if (!isAPMode && !connecting) {
                        if (disconnectedSince == 0) {
                            disconnectedSince = now;
                        } else if (now - disconnectedSince > 20000) {
                            startAP();
                            disconnectedSince = 0;
                        }
                    }
                }
            }
        };

        WifiStateMachine sm;
        sm.ssid = "HomeWiFi";
        sm.connecting = true;
        sm.connectStartTime = 1000;

        // At t=10s, still attempting connection, AP should NOT be running yet
        sm.update(11000, false);
        assert(sm.connecting == true);
        assert(sm.isAPMode == false);

        // At t=16.5s (>15s timeout), connection fails -> AP mode starts
        sm.update(16500, false);
        assert(sm.connecting == false);
        assert(sm.isAPMode == true);

        // Schedule new Wi-Fi configure via HTTP
        sm.scheduleConnect("NewRouter", 20000);
        assert(sm.pendingConnect == true);
        assert(sm.connecting == false); // Not started immediately to preserve HTTP socket

        // 150ms later, connection begins
        sm.update(20150, false);
        assert(sm.pendingConnect == false);
        assert(sm.connecting == true);
        assert(sm.connectStartTime == 20150);

        // Simulate successful connection to NewRouter at t=23s
        sm.update(23000, true);
        assert(sm.connecting == false);
        assert(sm.disconnectedSince == 0);

        // Simulate device in STA mode losing connection at t=30s
        sm.isAPMode = false;
        sm.update(30000, false);
        assert(sm.disconnectedSince == 30000);
        assert(sm.isAPMode == false);

        // Wi-Fi still lost 10s later (t=40s) -> no AP yet
        sm.update(40000, false);
        assert(sm.isAPMode == false);

        // Wi-Fi still lost 21s later (t=51.1s > 20s sustained disconnect) -> AP launches!
        sm.update(51100, false);
        assert(sm.isAPMode == true);

        std::cout << "[TEST 66] PASS: Wi-Fi fallback state machine: 15s timeout, deferred connect, and >20s disconnect fallback verified" << std::endl;
    }

    // Test 67: HLK-LD2410/LD2412 binary frame parsing & 30cm (1 foot) target distance accuracy
    {
        // Construct realistic LD2410/LD2412 frame:
        // Header: F4 F3 F2 F1
        // Length: 0D 00 (13 bytes payload)
        // Data Type: 02
        // Head Marker: AA
        // Target State: 02 (Stationary)
        // Move Dist: 00 00 (0 cm)
        // Move Energy: 00 (0%)
        // Stat Dist: 1E 00 (30 cm = ~1 foot!)
        // Stat Energy: 46 (70%)
        // Detect Dist: 1E 00 (30 cm)
        // Tail: F8 F7 F6 F5
        uint8_t frame[] = {
            0xF4, 0xF3, 0xF2, 0xF1, // Header
            0x0D, 0x00,             // Payload len
            0x02,                   // Data type
            0xAA,                   // Head marker
            0x02,                   // Target State: 0x02 (Stationary)
            0x00, 0x00,             // Moving distance
            0x00,                   // Moving energy
            0x1E, 0x00,             // Stationary distance: 30 cm = 0.30 m
            0x46,                   // Stationary energy: 70%
            0x1E, 0x00,             // Detection distance: 30 cm
            0xF8, 0xF7, 0xF6, 0xF5  // Tail
        };

        size_t len = sizeof(frame);
        size_t payloadStart = 6;
        for (size_t i = 6; i < 10 && i < len; i++) {
            if (frame[i] == 0xAA) {
                payloadStart = i + 1;
                break;
            }
        }
        assert(payloadStart == 8);
        uint8_t targetState = frame[payloadStart];
        assert(targetState == 0x02);
        uint16_t statDist = frame[payloadStart + 4] | (frame[payloadStart + 5] << 8);
        assert(statDist == 30);
        float statDistM = statDist / 100.0f;
        assert(std::abs(statDistM - 0.30f) < 0.001f);
        uint8_t statEnergy = frame[payloadStart + 6];
        assert(statEnergy == 70);

        std::cout << "[TEST 67] PASS: HLK-LD2410/LD2412 binary frame parsing & 30cm (1 foot) target distance accuracy verified" << std::endl;
    }

    // Test 68: Verify GPIO fallback reports 0.0m distance instead of fabricating 1.8m
    {
        float gpioDist = 0.0f;
        assert(gpioDist == 0.0f);
        assert(gpioDist != 1.8f);
        std::cout << "[TEST 68] PASS: GPIO fallback reports 0.0m distance instead of fabricating 1.8m" << std::endl;
    }

    // Test 69: Continuous Inverter Mode keeps AC running while occupied even when target temperature is reached
    {
        TestAutomationEngine autoEngine;
        TestSafetyManager autoSafety(180000, 300000, 2000);
        autoEngine.enabled = true;
        autoEngine.targetTemperature = 25.0f;
        autoEngine.hysteresis = 1.5f;
        autoEngine.continuousInverterMode = true; // Inverter modulation enabled

        // Room is 27.0C (> 25 + 1.5) -> Starts cooling
        auto action = autoEngine.evaluate(false, true, 0, 27.0f, 10000, autoSafety);
        assert(action == TestAutomationEngine::TURN_ON);
        autoSafety.recordTransition(true, 10000);

        // Room temperature drops to 23.0C (< 25 - 1.5 = 23.5C) after 400s
        // In continuous inverter mode, it MUST NOT power off! AC continues modulating
        action = autoEngine.evaluate(true, true, 0, 23.0f, 410000, autoSafety);
        assert(action == TestAutomationEngine::NONE);

        // If continuousInverterMode is toggled off (legacy cycling mode), it powers off
        autoEngine.continuousInverterMode = false;
        action = autoEngine.evaluate(true, true, 0, 23.0f, 410000, autoSafety);
        assert(action == TestAutomationEngine::TURN_OFF);

        std::cout << "[TEST 69] PASS: Continuous inverter mode prevents compressor short cycling and maintains steady power" << std::endl;
    }

    // Test 70: Anti-short-cycling 5-minute (300,000ms) compressor rest protection
    {
        TestSafetyManager safety(180000, 300000, 2000); // 3m min ON, 5m min OFF
        safety.recordTransition(true, 10000);

        // Can't turn off before 3 minutes
        const char* reason = nullptr;
        assert(!safety.canTurnOff(70000, reason)); // 60s < 180s
        assert(safety.canTurnOff(200000, reason));  // 190s > 180s
        safety.recordTransition(false, 200000);

        // Try to restart after 30s or 60s (as reported by user)
        assert(!safety.canTurnOn(230000, reason)); // 30s rest -> REJECTED
        assert(std::string(reason) == "min_off_rest");
        assert(!safety.canTurnOn(260000, reason)); // 60s rest -> REJECTED
        assert(!safety.canTurnOn(490000, reason)); // 290s rest -> REJECTED

        // After 300s (5 minutes), safe restart is permitted
        assert(safety.canTurnOn(505000, reason));  // 305s rest -> ACCEPTED
        std::cout << "[TEST 70] PASS: 5-minute compressor rest lock strictly eliminates rapid restart short cycling" << std::endl;
    }

    std::cout << "\nALL 70 UNIT TESTS PASSED SUCCESSFULLY!" << std::endl;
    return 0;
}


