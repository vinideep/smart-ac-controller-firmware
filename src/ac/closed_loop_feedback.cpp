#include "closed_loop_feedback.h"
#include "../storage/storage_manager.h"

namespace ac::control {

ClosedLoopFeedback::ClosedLoopFeedback(AzureEssenceController& ac, sensors::EnergyMonitor& energy)
    : _ac(ac), _energy(energy) {}

void ClosedLoopFeedback::begin() {
    storage::ClosedLoopStorageConfig cfg;
    if (storage::StorageManager::loadClosedLoopConfig(cfg)) {
        _enabled = cfg.enabled;
        _verifyTimeoutMs = cfg.verifyTimeoutS * 1000;
        _maxRetries = cfg.maxRetries;
    }
}

void ClosedLoopFeedback::setEnabled(bool enabled) {
    _enabled = enabled;
    storage::ClosedLoopStorageConfig cfg;
    cfg.enabled = _enabled;
    cfg.verifyTimeoutS = _verifyTimeoutMs / 1000;
    cfg.maxRetries = _maxRetries;
    storage::StorageManager::saveClosedLoopConfig(cfg);
}

void ClosedLoopFeedback::setVerificationTimeoutSeconds(uint32_t sec) {
    if (sec >= 10 && sec <= 300) {
        _verifyTimeoutMs = sec * 1000;
        storage::ClosedLoopStorageConfig cfg;
        cfg.enabled = _enabled;
        cfg.verifyTimeoutS = sec;
        cfg.maxRetries = _maxRetries;
        storage::StorageManager::saveClosedLoopConfig(cfg);
    }
}

void ClosedLoopFeedback::setMaxRetries(uint8_t retries) {
    if (retries <= 5) {
        _maxRetries = retries;
        storage::ClosedLoopStorageConfig cfg;
        cfg.enabled = _enabled;
        cfg.verifyTimeoutS = _verifyTimeoutMs / 1000;
        cfg.maxRetries = retries;
        storage::StorageManager::saveClosedLoopConfig(cfg);
    }
}

void ClosedLoopFeedback::notifyCommandSent(const ACState& targetState, uint32_t nowMs) {
    if (!_enabled) return;
    uint32_t now = (nowMs == 0) ? millis() : nowMs;
    _expectedState = targetState;
    _commandSentTimeMs = now;
    _pendingVerification = true;
    _currentRetry = 0;
    _status = DesyncStatus::Verifying;
    if (targetState.power) {
        _coolingStartTimeMs = 0;
        _thermalStallDesync = false;
    } else {
        _coolingStartTimeMs = 0;
        _thermalStallDesync = false;
        _energy.setThermalStallSuspended(false);
    }
}

void ClosedLoopFeedback::update(uint32_t nowMs, float currentTemp) {
    if (!_enabled) return;
    uint32_t now = (nowMs == 0) ? millis() : nowMs;

    if (_pendingVerification) {
        uint32_t elapsed = now - _commandSentTimeMs;

        if (_expectedState.power) {
            // Target is ON: AC should pull fan or compressor power (> 20W)
            if (_energy.getPowerWatts() >= 20.0f) {
                _status = DesyncStatus::Synchronized;
                _pendingVerification = false;
                _currentRetry = 0;
                Serial.printf("[CLOSED_LOOP] AC ON confirmed by electrical load: %.1fW\n", _energy.getPowerWatts());
            } else if (elapsed >= _verifyTimeoutMs) {
                // AC power is still in Standby (< 20W)
                if (_currentRetry < _maxRetries) {
                    _currentRetry++;
                    _status = DesyncStatus::Retrying;
                    _commandSentTimeMs = now;
                    Serial.printf("[CLOSED_LOOP] Power mismatch (%.1fW in standby). Resending IR frame (%d/%d)...\n",
                                  _energy.getPowerWatts(), _currentRetry, _maxRetries);
                    _ac.sendState("closed_loop_resend");
                } else {
                    _status = DesyncStatus::DesyncDetected;
                    _pendingVerification = false;
                    Serial.printf("[CLOSED_LOOP] DESYNC ALERT: AC failed to turn ON after %d retries (power: %.1fW)\n",
                                  _maxRetries, _energy.getPowerWatts());
                }
            }
        } else {
            // Target is OFF: AC should drop to Standby (< 20W)
            if (_energy.getPowerWatts() < 20.0f) {
                _status = DesyncStatus::Synchronized;
                _pendingVerification = false;
                _currentRetry = 0;
                Serial.printf("[CLOSED_LOOP] AC OFF confirmed by electrical standby: %.1fW\n", _energy.getPowerWatts());
            } else if (elapsed >= 25000) {
                // AC power still active (> 20W) after 25s
                if (_currentRetry < _maxRetries) {
                    _currentRetry++;
                    _status = DesyncStatus::Retrying;
                    _commandSentTimeMs = now;
                    Serial.printf("[CLOSED_LOOP] Power mismatch (%.1fW still active). Resending IR POWER_OFF (%d/%d)...\n",
                                  _energy.getPowerWatts(), _currentRetry, _maxRetries);
                    _ac.setPower(false, "closed_loop_resend_off");
                } else {
                    _status = DesyncStatus::DesyncDetected;
                    _pendingVerification = false;
                    Serial.printf("[CLOSED_LOOP] DESYNC ALERT: AC failed to turn OFF after %d retries (power: %.1fW)\n",
                                  _maxRetries, _energy.getPowerWatts());
                }
            }
        }
    } else {
        // Zero-Desync: Detect external physical remote actuation
        const ACState& currentState = _ac.getState();

        if (!currentState.power) {
            // Internal state is OFF, but power draws active compressor (> 250W)
            if (_energy.isCompressorActive()) {
                if (_extHighPowerStart == 0) _extHighPowerStart = now;
                if (now - _extHighPowerStart >= 8000) {
                    _ac.applyExternalPower(true);
                    _extHighPowerStart = 0;
                    _status = DesyncStatus::Synchronized;
                    Serial.println("[CLOSED_LOOP] External activation detected (>250W sustained): state synced to ON");
                }
            } else {
                _extHighPowerStart = 0;
            }
        } else {
            // Internal state is ON, but power dropped to idle standby (< 15W)
            if (_energy.isStandby()) {
                if (_extStandbyStart == 0) _extStandbyStart = now;
                if (now - _extStandbyStart >= 20000) {
                    _ac.applyExternalPower(false);
                    _extStandbyStart = 0;
                    _status = DesyncStatus::Synchronized;
                    Serial.println("[CLOSED_LOOP] External shutdown detected (<15W idle): state synced to OFF");
                }
            } else {
                _extStandbyStart = 0;
            }
        }
    }

    // Thermal rate (dT/dt) monitoring: detect if AC commanded ON fails to cool room after 8 mins
    if (_ac.getState().power) {
        if (currentTemp > -50.0f) {
            if (_coolingStartTimeMs == 0) {
                _coolingStartTimeMs = now;
                _coolingStartTemp = currentTemp;
            } else {
                uint32_t elapsedCooling = now - _coolingStartTimeMs;
                if (elapsedCooling >= 480000) { // 8 minutes (480s)
                    if (currentTemp >= _coolingStartTemp) {
                        _status = DesyncStatus::ThermalStallDesync;
                        if (!_thermalStallDesync) {
                            _thermalStallDesync = true;
                            _energy.setThermalStallSuspended(true);
                            Serial.printf("[CLOSED_LOOP] THERMAL STALL DESYNC: AC ON for 8m but room failed to cool (%.1fC -> %.1fC). Suspending synthetic energy.\n",
                                          _coolingStartTemp, currentTemp);
                        }
                    } else if (_thermalStallDesync && currentTemp < _coolingStartTemp - 0.5f) {
                        _thermalStallDesync = false;
                        _energy.setThermalStallSuspended(false);
                        _status = DesyncStatus::Synchronized;
                        _coolingStartTimeMs = now;
                        _coolingStartTemp = currentTemp;
                        Serial.println("[CLOSED_LOOP] Thermal stall resolved. Resumed energy accumulation.");
                    }
                }
            }
        }
    } else {
        _coolingStartTimeMs = 0;
        if (_thermalStallDesync) {
            _thermalStallDesync = false;
            _energy.setThermalStallSuspended(false);
            if (_status == DesyncStatus::ThermalStallDesync) {
                _status = DesyncStatus::Synchronized;
            }
        }
    }
}

void ClosedLoopFeedback::clearThermalStallDesync() {
    _thermalStallDesync = false;
    _coolingStartTimeMs = 0;
    _energy.setThermalStallSuspended(false);
    if (_status == DesyncStatus::ThermalStallDesync) {
        _status = DesyncStatus::Synchronized;
    }
}

const char* ClosedLoopFeedback::getStatusStr() const {
    switch (_status) {
        case DesyncStatus::Verifying: return "verifying";
        case DesyncStatus::Retrying: return "retrying";
        case DesyncStatus::DesyncDetected: return "desync_detected";
        case DesyncStatus::ThermalStallDesync: return "thermal_stall_desync";
        case DesyncStatus::Synchronized:
        default: return "synchronized";
    }
}

void ClosedLoopFeedback::toJSON(JsonDocument& doc) const {
    doc["enabled"] = _enabled;
    doc["status"] = getStatusStr();
    doc["thermal_stall_desync"] = _thermalStallDesync;
    doc["pending_verification"] = _pendingVerification;
    doc["retry_count"] = _currentRetry;
    doc["max_retries"] = _maxRetries;
    doc["verify_timeout_s"] = _verifyTimeoutMs / 1000;
}

} // namespace ac::control
