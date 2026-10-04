#include "safety_manager.h"

namespace ac::safety {

SafetyManager::SafetyManager(uint32_t minOnTimeMs, uint32_t minOffTimeMs, uint32_t minStateDelayMs)
    : _minOnTimeMs(minOnTimeMs),
      _minOffTimeMs(minOffTimeMs),
      _minStateDelayMs(minStateDelayMs) {}

uint32_t SafetyManager::getOnDurationMs(uint32_t nowMs) const {
    if (!_isAcPowered || _lastTurnOnMs == 0) return 0;
    return (nowMs - _lastTurnOnMs);
}

uint32_t SafetyManager::getOffDurationMs(uint32_t nowMs) const {
    if (_isAcPowered) return 0;
    if (_lastTurnOffMs == 0) return 0xFFFFFFFF; // Never turned off yet, safe to turn on
    return (nowMs - _lastTurnOffMs);
}


bool SafetyManager::canTurnOn(uint32_t nowMs, const char*& rejectReason) const {
    if (_isAcPowered) {
        rejectReason = "AC is already powered ON";
        return false;
    }
    if (_lastTurnOffMs > 0 && (nowMs - _lastTurnOffMs) < _minOffTimeMs) {
        rejectReason = "Compressor protection: minimum OFF rest time has not elapsed";
        return false;
    }
    if (_lastCommandMs > 0 && (nowMs - _lastCommandMs) < _minStateDelayMs) {
        rejectReason = "Rate limit: please wait between IR transmissions";
        return false;
    }
    rejectReason = nullptr;
    return true;
}

bool SafetyManager::canTurnOff(uint32_t nowMs, const char*& rejectReason) const {
    if (_lastTurnOnMs > 0 && _isAcPowered && (nowMs - _lastTurnOnMs) < _minOnTimeMs) {
        rejectReason = "Compressor protection: minimum ON run time has not elapsed";
        return false;
    }
    if (_lastCommandMs > 0 && (nowMs - _lastCommandMs) < _minStateDelayMs) {
        rejectReason = "Rate limit: please wait between IR transmissions";
        return false;
    }
    rejectReason = nullptr;
    return true;
}

bool SafetyManager::canSendState(uint32_t nowMs, const char*& rejectReason) const {
    if (_lastCommandMs > 0 && (nowMs - _lastCommandMs) < _minStateDelayMs) {
        rejectReason = "Rate limit: commands throttled to prevent IR collision";
        return false;
    }
    rejectReason = nullptr;
    return true;
}

void SafetyManager::recordPowerTransition(bool poweredOn, uint32_t nowMs) {
    _isAcPowered = poweredOn;
    if (poweredOn) {
        _lastTurnOnMs = nowMs;
    } else {
        _lastTurnOffMs = nowMs;
    }
    _lastCommandMs = nowMs;
}

void SafetyManager::recordCommandSent(uint32_t nowMs) {
    _lastCommandMs = nowMs;
}

} // namespace ac::safety
