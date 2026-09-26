#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include "../ac/ac_controller.h"
#include "../safety/safety_manager.h"
#include "../sensors/presence_sensor.h"
#include "../sensors/dht_sensor.h"

namespace ac::automation {

enum class PresenceTier {
    Active,
    EcoDrift,
    Vacant,
    WelcomeBack
};

enum class SleepStage {
    Inactive,
    Pulldown,
    DeepSleepRamp,
    RemHold,
    Wakeup
};

struct CircadianSleepConfig {
    bool enabled = false;
    uint8_t pulldownTemp = 23;
    float rampRatePerHr = 0.5f;
    float maxRampTemp = 25.5f;
    uint32_t sleepStartTimeMs = 0;
};

struct AutomationConfig {
    bool enabled = false;
    float targetTemperature = 25.0f;
    float baseTargetTemp = 25.0f;
    float hysteresis = 1.0f;
    uint32_t ecoDriftTimeoutSeconds = 300;   // 5 minutes
    uint32_t emptyTimeoutSeconds = 900;       // 15 minutes
    uint32_t evalIntervalMs = 5000;           // Evaluate every 5 seconds

    bool ecoDriftEnabled = true;
    bool psychrometricEnabled = true;
    float dryModeHumidityThreshold = 65.0f;
    bool thermalBreachProtection = true;
    float breachRiseThreshold = 1.2f;
    uint32_t breachWindowMs = 180000;         // 3 minutes (180s)
    bool presenceDetectionEnabled = false;    // When false, regulates climate without waiting for presence sensor

    CircadianSleepConfig sleepConfig;
};

struct ThermalSample {
    float temp = 0.0f;
    uint32_t timeMs = 0;
};

class AutomationEngine {
public:
    AutomationEngine(control::ACController& ac,
                     safety::SafetyManager& safety,
                     sensors::PresenceSensorDriver& presence,
                     DHTDriver& dht);

    void begin();
    void update();

    void setEnabled(bool enable);
    bool isEnabled() const { return _config.enabled; }

    void setTargetTemperature(float target);
    float getTargetTemperature() const { return _config.targetTemperature; }
    float getBaseTargetTemperature() const { return _config.baseTargetTemp; }

    void setHysteresis(float hyst);
    float getHysteresis() const { return _config.hysteresis; }

    void setEmptyTimeoutSeconds(uint32_t seconds);
    uint32_t getEmptyTimeoutSeconds() const { return _config.emptyTimeoutSeconds; }

    void setEcoDriftEnabled(bool enable);
    bool isEcoDriftEnabled() const { return _config.ecoDriftEnabled; }

    void setEcoDriftTimeoutSeconds(uint32_t seconds);
    uint32_t getEcoDriftTimeoutSeconds() const { return _config.ecoDriftTimeoutSeconds; }

    void setPsychrometricEnabled(bool enable);
    bool isPsychrometricEnabled() const { return _config.psychrometricEnabled; }

    void setDryModeHumidityThreshold(float threshold);
    float getDryModeHumidityThreshold() const { return _config.dryModeHumidityThreshold; }

    void setThermalBreachProtection(bool enable);
    bool isThermalBreachProtection() const { return _config.thermalBreachProtection; }
    bool isThermalBreachActive() const { return _thermalBreachActive; }
    float getThermalBreachDelta() const { return _thermalBreachDelta; }
    void clearThermalBreach() {
        _thermalBreachActive = false;
        _thermalBreachDelta = 0.0f;
        _breachBufCount = 0;
        _breachBufHead = 0;
    }

    void setPresenceDetectionEnabled(bool enable);
    bool isPresenceDetectionEnabled() const { return _config.presenceDetectionEnabled; }

    void loadFromNVS();
    void saveToNVS();

    void setCircadianSleep(bool enable, uint8_t pulldown = 23, float ramp = 0.5f, float maxTemp = 25.5f);
    bool isCircadianSleepEnabled() const { return _config.sleepConfig.enabled; }
    SleepStage getSleepStage() const { return _sleepStage; }
    const char* getSleepStageStr() const;

    PresenceTier getPresenceTier() const { return _presenceTier; }
    const char* getPresenceTierStr() const;

    void toJSON(JsonDocument& doc) const;

private:
    control::ACController& _ac;
    safety::SafetyManager& _safety;
    sensors::PresenceSensorDriver& _presence;
    DHTDriver& _dht;

    AutomationConfig _config;
    uint32_t _lastEvalTime = 0;

    PresenceTier _presenceTier = PresenceTier::Active;
    SleepStage _sleepStage = SleepStage::Inactive;
    bool _lastPresenceState = true;
    bool _turnedOffByVacancy = false;
    bool _thermalBreachActive = false;
    float _thermalBreachDelta = 0.0f;

    // Rolling buffer for breach detection (sampled every 10s up to 24 samples = 4 mins)
    static constexpr size_t kBreachBufferSize = 24;
    ThermalSample _breachBuffer[kBreachBufferSize];
    size_t _breachBufCount = 0;
    size_t _breachBufHead = 0;
    uint32_t _lastBreachSampleTime = 0;

    void recordBreachSample(float temp, uint32_t now);
    void evaluateThermalBreach(float currentTemp, uint32_t now, bool acOn);
    void evaluateCircadianSleep(uint32_t now);
    void logEvent(const char* action, const char* reason, float currentTemp);
};

} // namespace ac::automation
