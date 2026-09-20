#include "ir_transmitter.h"
#include <ArduinoJson.h>

namespace ac::ir {

IRTransmitterDriver::IRTransmitterDriver(uint8_t txPin)
    : _txPin(txPin), _irsend(txPin) {}

void IRTransmitterDriver::begin() {
    if (!_initialized) {
        _irsend.begin();
        _initialized = true;
    }
}

bool IRTransmitterDriver::sendRaw(const uint16_t* rawData, uint16_t length, uint16_t freqKhz) {
    if (!_initialized) begin();
    if (rawData == nullptr || length == 0) return false;

    _irsend.sendRaw(rawData, length, freqKhz);
    _txCount++;
    logTransmission("send_raw", true);
    return true;
}

bool IRTransmitterDriver::sendProtocol(const uint8_t stateBytes[kAzureStateLength], uint16_t repeats) {
    if (!_initialized) begin();

    uint16_t rawBuf[kAzureRawTransitions];
    uint16_t count = AzureEssenceProtocol::generateRaw(stateBytes, rawBuf, kAzureRawTransitions);
    if (count == 0) return false;

    // First transmission
    _irsend.sendRaw(rawBuf, count, 38);

    // Any repeat frames separated by 21ms gap
    for (uint16_t r = 0; r < repeats; r++) {
        delayMicroseconds(kAzureRepeatSpace);
        _irsend.sendRaw(rawBuf, count, 38);
    }

    _txCount++;
    return true;
}

bool IRTransmitterDriver::sendAcState(const control::ACState& state, uint16_t repeats) {
    uint8_t stateBytes[kAzureStateLength];
    AzureEssenceProtocol::encode(state, stateBytes);

    bool ok = sendProtocol(stateBytes, repeats);
    logTransmission("send_ac_state", ok, &state);
    return ok;
}

bool IRTransmitterDriver::testPowerOn() {
    control::ACState state;
    state.power = true;
    state.temperature = 25;
    state.mode = "cool";
    state.fanSpeed = "auto";
    state.source = "test_power_on";
    return sendAcState(state, 1); // 1 repeat frame for rock-solid reception
}

bool IRTransmitterDriver::testPowerOff() {
    control::ACState state;
    state.power = false;
    state.temperature = 25;
    state.mode = "cool";
    state.fanSpeed = "auto";
    state.source = "test_power_off";
    return sendAcState(state, 1); // 1 repeat frame
}

void IRTransmitterDriver::logTransmission(const char* command, bool success, const control::ACState* state) {
    JsonDocument doc;
    doc["type"] = "ir_transmit";
    doc["command"] = command;
    doc["success"] = success;
    doc["tx_count"] = _txCount;
    doc["pin"] = _txPin;
    doc["timestamp_ms"] = millis();

    if (state != nullptr) {
        doc["power"] = state->power;
        doc["temperature"] = state->temperature;
        doc["mode"] = state->mode;
        doc["fan_speed"] = state->fanSpeed;
        doc["source"] = state->source;
    }

    String output;
    serializeJson(doc, output);
    Serial.println(output);
}

} // namespace ac::ir
