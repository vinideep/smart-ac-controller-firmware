#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include "../ac/ac_controller.h"
#include "../safety/safety_manager.h"
#include "../sensors/presence_sensor.h"
#include "../sensors/dht_sensor.h"

namespace ac::automation {

struct AutomationConfig {
    bool enabled = false;
    float targetTemperature = 25.0f;
    float hysteresis = 1.0f;
    uint32_t emptyTimeoutSeconds = 900; // 15 minutes
    uint32_t evalIntervalMs = 5000;     // Evaluate every 5 seconds
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

    void setHysteresis(float hyst);
    float getHysteresis() const { return _config.hysteresis; }

    void setEmptyTimeoutSeconds(uint32_t seconds);
    uint32_t getEmptyTimeoutSeconds() const { return _config.emptyTimeoutSeconds; }

    void toJSON(JsonDocument& doc) const;

private:
    control::ACController& _ac;
    safety::SafetyManager& _safety;
    sensors::PresenceSensorDriver& _presence;
    DHTDriver& _dht;

    AutomationConfig _config;
    uint32_t _lastEvalTime = 0;

    void logEvent(const char* action, const char* reason, float currentTemp);
};

} // namespace ac::automation
