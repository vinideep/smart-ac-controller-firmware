#include "ac_controller.h"
#include "../storage/storage_manager.h"

namespace ac::control {

AzureEssenceController::AzureEssenceController(uint8_t txPin)
    : _transmitter(txPin) {
    _state.power = false;
    _state.temperature = 25;
    _state.mode = "cool";
    _state.fanSpeed = "auto";
    _state.swing = false;
    _state.sleep = false;
    _state.source = "init";
    _state.timestamp = 0;
}

void AzureEssenceController::begin() {
    _transmitter.begin();
    restoreStateFromStorage();
}

void AzureEssenceController::restoreStateFromStorage() {
    if (storage::StorageManager::loadAcState(_state)) {
        Serial.printf("[NVS] Restored AC state: Power=%s Temp=%dC Mode=%s Fan=%s\n",
                      _state.power ? "ON" : "OFF", _state.temperature, _state.mode.c_str(), _state.fanSpeed.c_str());
    }
}

bool AzureEssenceController::validateTemperature(uint8_t tempC) const {
    return (tempC >= kMinTemperature && tempC <= kMaxTemperature);
}

bool AzureEssenceController::validateMode(const String& mode) const {
    return (mode == "cool" || mode == "fan" || mode == "dry" || mode == "auto");
}

bool AzureEssenceController::validateFanSpeed(const String& fanSpeed) const {
    return (fanSpeed == "auto" || fanSpeed == "med" || fanSpeed == "low" || fanSpeed == "high");
}

bool AzureEssenceController::setPower(bool power, const String& source) {
    _state.power = power;
    _state.source = source;
    _state.timestamp = millis();
    return sendState(source);
}

bool AzureEssenceController::setMode(const String& mode, const String& source) {
    if (!validateMode(mode)) return false;
    _state.mode = mode;
    _state.source = source;
    _state.timestamp = millis();
    return sendState(source);
}

bool AzureEssenceController::setTemperature(uint8_t tempC, const String& source) {
    if (!validateTemperature(tempC)) return false;
    _state.temperature = tempC;
    _state.source = source;
    _state.timestamp = millis();
    return sendState(source);
}

bool AzureEssenceController::setFanSpeed(const String& fanSpeed, const String& source) {
    if (!validateFanSpeed(fanSpeed)) return false;
    _state.fanSpeed = fanSpeed;
    _state.source = source;
    _state.timestamp = millis();
    return sendState(source);
}

bool AzureEssenceController::setSwing(bool swing, const String& source) {
    _state.swing = swing;
    _state.source = source;
    _state.timestamp = millis();
    return sendState(source);
}

bool AzureEssenceController::setSleep(bool sleep, const String& source) {
    _state.sleep = sleep;
    _state.source = source;
    _state.timestamp = millis();
    return sendState(source);
}

bool AzureEssenceController::setState(bool power, uint8_t tempC, const String& mode, const String& fanSpeed, const String& source) {
    if (!validateTemperature(tempC)) return false;
    if (!validateMode(mode)) return false;
    if (!validateFanSpeed(fanSpeed)) return false;
    _state.power = power;
    _state.temperature = tempC;
    _state.mode = mode;
    _state.fanSpeed = fanSpeed;
    _state.source = source;
    _state.timestamp = millis();
    return sendState(source);
}

bool AzureEssenceController::sendState(const String& source) {
    _state.source = source;
    _state.timestamp = millis();
    storage::StorageManager::saveAcState(_state);
    // Transmit with 1 repeat frame for guaranteed reception
    return _transmitter.sendAcState(_state, 1);
}

void AzureEssenceController::applyExternalState(const ACState& newState) {
    _state.power = newState.power;
    _state.temperature = newState.temperature;
    _state.mode = newState.mode;
    _state.fanSpeed = newState.fanSpeed;
    _state.source = "ir_remote";
    _state.timestamp = millis();
    storage::StorageManager::saveAcState(_state);
    Serial.printf("[AC_MIRROR] State updated from remote IR: Power=%s Temp=%dC Mode=%s Fan=%s\n",
                  _state.power ? "ON" : "OFF", _state.temperature, _state.mode.c_str(), _state.fanSpeed.c_str());
}

void AzureEssenceController::applyExternalPower(bool power) {
    _state.power = power;
    _state.source = "closed_loop_sync";
    _state.timestamp = millis();
    storage::StorageManager::saveAcState(_state);
    Serial.printf("[AC_MIRROR] Power state synchronized from electrical draw: Power=%s\n",
                  _state.power ? "ON" : "OFF");
}

} // namespace ac::control
