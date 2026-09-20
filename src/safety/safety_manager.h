#pragma once

#include <stdint.h>
#include <stddef.h>
#include <Arduino.h>

namespace ac::safety {

class SafetyManager {
public:
    explicit SafetyManager(uint32_t minOnTimeMs = 180000,    // 3 minutes min compressor runtime
                           uint32_t minOffTimeMs = 180000,   // 3 minutes min compressor rest time
                           uint32_t minStateDelayMs = 5000); // 5 seconds between consecutive commands

    /**
     * @brief Evaluates if turning the AC ON is permitted under compressor safety rules.
     */
    bool canTurnOn(uint32_t nowMs, const char*& rejectReason) const;

    /**
     * @brief Evaluates if turning the AC OFF is permitted under compressor safety rules.
     */
    bool canTurnOff(uint32_t nowMs, const char*& rejectReason) const;

    /**
     * @brief Evaluates if a general state update (temp/fan change) is rate-limited.
     */
    bool canSendState(uint32_t nowMs, const char*& rejectReason) const;

    /**
     * @brief Records a committed state change to update safety timers.
     */
    void recordPowerTransition(bool poweredOn, uint32_t nowMs);
    void recordCommandSent(uint32_t nowMs);

    bool isAcPowered() const { return _isAcPowered; }
    uint32_t getLastTurnOnTime() const { return _lastTurnOnMs; }
    uint32_t getLastTurnOffTime() const { return _lastTurnOffMs; }
    uint32_t getOnDurationMs(uint32_t nowMs) const;
    uint32_t getOffDurationMs(uint32_t nowMs) const;

private:
    uint32_t _minOnTimeMs;
    uint32_t _minOffTimeMs;
    uint32_t _minStateDelayMs;

    bool _isAcPowered = false;
    uint32_t _lastTurnOnMs = 0;
    uint32_t _lastTurnOffMs = 0;
    uint32_t _lastCommandMs = 0;
};

} // namespace ac::safety
