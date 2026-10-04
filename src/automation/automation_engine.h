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

enum class NightCycleStage {
    Inactive,
    InitialPulldown,         // Phase 1: Cool to 27°C
    Pause1_30m,              // Turn OFF for 30 minutes
    Cycle2_Cool27,           // Phase 2: Cool to 27°C again
    Pause2_40m,              // Turn OFF for 40 minutes
    Midnight_Cool27,         // Phase 3: Midnight cool to 27°C
    Midnight_PauseExtended,  // Mid-night extended OFF (75 mins)
    PreDawn_Burst20m,        // Phase 4: 3am-5am window 20m burst ON
    PreDawn_Pause2h,         // Turn OFF for 2 hours (120 mins)
    Completed                // Night cycle complete
};

struct CircadianSleepConfig {
    bool enabled = false;
    uint8_t pulldownTemp = 23;
    float rampRatePerHr = 0.5f;
    float maxRampTemp = 25.5f;
    uint32_t sleepStartTimeMs = 0;
};

struct NightCycleConfig {
    bool enabled = false;
    float targetTemp = 27.0f;
    uint32_t pause1DurationSec = 1800;   // 30 mins
    uint32_t pause2DurationSec = 2400;   // 40 mins
    uint32_t midnightPauseSec = 4500;    // 75 mins (adjusted by location clock)
    uint32_t preDawnBurstSec = 1200;     // 20 mins
    uint32_t preDawnPauseSec = 7200;     // 2 hours
    uint32_t stageStartTimeMs = 0;
    uint8_t startHour = 22;              // Hour when activated (defaults to 10 PM)
    uint8_t startMinute = 0;
    bool preDawnTriggered = false;
};

struct AutomationConfig {
    bool enabled = false;
    float targetTemperature = 25.0f;
    float baseTargetTemp = 25.0f;
    float hysteresis = 1.5f;
    uint32_t ecoDriftTimeoutSeconds = 300;   // 5 minutes
    uint32_t emptyTimeoutSeconds = 900;       // 15 minutes
    uint32_t evalIntervalMs = 5000;           // Evaluate every 5 seconds

    bool ecoDriftEnabled = true;
    bool psychrometricEnabled = true;
    bool comfortIndexOptimization = true;
    float dryModeHumidityThreshold = 65.0f;
    bool thermalBreachProtection = true;
    float breachRiseThreshold = 1.2f;
    uint32_t breachWindowMs = 180000;         // 3 minutes (180s)
    bool presenceDetectionEnabled = false;    // When false, regulates climate without waiting for presence sensor
    bool continuousInverterMode = true;       // Keeps AC running and modulates setpoint instead of hard power cycling

    CircadianSleepConfig sleepConfig;
    NightCycleConfig nightCycle;
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

    void setComfortIndexOptimization(bool enable);
    bool isComfortIndexOptimization() const { return _config.comfortIndexOptimization; }

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

    void setContinuousInverterMode(bool enable);
    bool isContinuousInverterMode() const { return _config.continuousInverterMode; }

    void loadFromNVS();
    void saveToNVS();

    void setCircadianSleep(bool enable, uint8_t pulldown = 23, float ramp = 0.5f, float maxTemp = 25.5f);
    bool isCircadianSleepEnabled() const { return _config.sleepConfig.enabled; }
    SleepStage getSleepStage() const { return _sleepStage; }
    const char* getSleepStageStr() const;

    void syncTime(uint32_t epochSec, int16_t tzOffsetMin = 330);
    bool getLocalTime(uint8_t& outHour, uint8_t& outMinute, uint8_t& outSecond) const;
    uint8_t getLocalHour() const;
    uint8_t getLocalMinute() const;
    bool hasValidTime() const;

    void setNightCycle(bool enable, float targetTemp = 27.0f, int8_t overrideHour = -1, int8_t overrideMin = -1);
    bool isNightCycleEnabled() const { return _config.nightCycle.enabled; }
    NightCycleStage getNightCycleStage() const { return _nightCycleStage; }
    const char* getNightCycleStageStr() const;
    uint32_t getNightCycleStageRemainingSec() const;

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
    NightCycleStage _nightCycleStage = NightCycleStage::Inactive;
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
    void evaluateNightCycle(uint32_t now);
    void logEvent(const char* action, const char* reason, float currentTemp);
};

} // namespace ac::automation
