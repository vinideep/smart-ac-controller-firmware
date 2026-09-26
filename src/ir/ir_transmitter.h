#pragma once

#include <Arduino.h>
#include <IRremoteESP8266.h>
#include <IRsend.h>
#include "ir_protocol_azure.h"
#include "../ac/ac_state.h"

namespace ac::ir {

class IRReceiverDriver;

class IRTransmitterDriver {
public:
    explicit IRTransmitterDriver(uint8_t txPin);

    void begin();

    void setReceiver(IRReceiverDriver* receiver) { _receiver = receiver; }
    uint32_t getLastTxTime() const { return _lastTxTime; }
    uint32_t getLastTxEndTime() const { return _lastTxTime; }
    bool isTransmitting() const { return _isTransmitting; }

    /**
     * @brief Transmit raw pulse timings at the specified modulation frequency.
     */
    bool sendRaw(const uint16_t* rawData, uint16_t length, uint16_t freqKhz = 38);

    /**
     * @brief Transmit a 9-byte state array using the Azure Essence pulse timings.
     */
    bool sendProtocol(const uint8_t stateBytes[kAzureStateLength], uint16_t repeats = 0);

    /**
     * @brief Encode and transmit a full AC state.
     */
    bool sendAcState(const control::ACState& state, uint16_t repeats = 0);

    /**
     * @brief Quick helper to send Power ON (Cool, 25°C, Auto Fan).
     */
    bool testPowerOn();

    /**
     * @brief Quick helper to send Power OFF.
     */
    bool testPowerOff();

    uint8_t getPin() const { return _txPin; }
    uint32_t getTxCount() const { return _txCount; }

private:
    uint8_t _txPin;
    IRsend _irsend;
    bool _initialized = false;
    uint32_t _txCount = 0;
    uint32_t _lastTxTime = 0;
    bool _isTransmitting = false;
    IRReceiverDriver* _receiver = nullptr;

    void pauseReceiver();
    void resumeReceiver();
    void logTransmission(const char* command, bool success, const control::ACState* state = nullptr);
};

} // namespace ac::ir
