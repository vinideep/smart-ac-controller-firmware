#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include "ac_state.h"
#include "ac_controller.h"
#include "../sensors/energy_monitor.h"

namespace ac::control {

enum class DesyncStatus {
    Synchronized,
    Verifying,
    Retrying,
    DesyncDetected,
    ThermalStallDesync
};

class ClosedLoopFeedback {
public:
    ClosedLoopFeedback(AzureEssenceController& ac, sensors::EnergyMonitor& energy);

    void begin();
    void update(uint32_t nowMs = 0, float currentTemp = -999.0f);

    void notifyCommandSent(const ACState& targetState, uint32_t nowMs = 0);

    void setEnabled(bool enabled);
    bool isEnabled() const { return _enabled; }

    void setVerificationTimeoutSeconds(uint32_t sec);
    uint32_t getVerificationTimeoutSeconds() const { return _verifyTimeoutMs / 1000; }

    void setMaxRetries(uint8_t retries);
    uint8_t getMaxRetries() const { return _maxRetries; }

    DesyncStatus getStatus() const { return _status; }
    const char* getStatusStr() const;
    uint8_t getRetryCount() const { return _currentRetry; }

    bool isThermalStallDesync() const { return _thermalStallDesync; }
    void clearThermalStallDesync();

    void toJSON(JsonDocument& doc) const;

private:
    AzureEssenceController& _ac;
    sensors::EnergyMonitor& _energy;

    bool _enabled = true;
    DesyncStatus _status = DesyncStatus::Synchronized;
    bool _pendingVerification = false;
    ACState _expectedState;
    uint32_t _commandSentTimeMs = 0;
    uint32_t _verifyTimeoutMs = 45000; // 45s default for compressor ramp-up
    uint8_t _maxRetries = 2;
    uint8_t _currentRetry = 0;

    uint32_t _coolingStartTimeMs = 0;
    float _coolingStartTemp = 0.0f;
    bool _thermalStallDesync = false;

    uint32_t _extHighPowerStart = 0;
    uint32_t _extStandbyStart = 0;
};

} // namespace ac::control
