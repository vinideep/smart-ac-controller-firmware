#pragma once
#include <Arduino.h>
#include <IRrecv.h>
#include <IRutils.h>
#include <IRac.h>
#include <memory>
#include "../config/pins.h"
#include "../ac/ac_state.h"

namespace ac::ir {

class IRTransmitterDriver;

constexpr uint16_t kCaptureBufferSize = 1024;
constexpr uint8_t  kTimeout           = 50;  // 50ms timeout for AC multi-burst frames
constexpr uint16_t kMinNoiseThreshold = 4;   // Filter spurious transitions / electrical noise

struct IRCaptureSummary {
    String protocol;
    int16_t protocolNum = -1;
    uint16_t bits = 0;
    String hexCode;
    bool repeat = false;
    bool overflow = false;
    bool isAc = false;
    uint16_t rawLength = 0;
    uint32_t timestampMs = 0;
};

struct IRCaptureInfo {
    bool hasData = false;
    String protocol;
    int16_t protocolNum = -1;
    uint16_t bits = 0;
    String hexCode;
    bool isAc = false;
    bool hasAcState = false;
    control::ACState acState;
    uint16_t rawLength = 0;
    uint16_t rawData[200] = {0};
    uint32_t timestampMs = 0;
};

class IRReceiverDriver {
public:
    explicit IRReceiverDriver(uint8_t pin = IR_RX_PIN,
                              uint16_t bufferSize = kCaptureBufferSize,
                              uint8_t timeout = kTimeout);

    void begin();
    bool update(const String& deviceId);
    void pause();
    void resume();
    void enable();
    void disable();
    IRrecv& getIRrecv() { return _irrecv; }

    void setTransmitter(IRTransmitterDriver* transmitter) { _transmitter = transmitter; }
    void setBlankingWindow(uint32_t windowMs) { _blankingWindowMs = windowMs; }
    void markTxBlanking(uint32_t timestampMs) { _lastTxBlankingMs = timestampMs; }

    static IRReceiverDriver* getInstance() { return s_instance; }

    const decode_results& getLastResults() const { return _results; }
    uint32_t getCaptureCount() const { return _captureCount; }
    uint8_t getPin() const { return _pin; }
    uint16_t getBufferSize() const { return _bufferSize; }
    uint8_t getTimeout() const { return _timeout; }
    uint16_t getRawLen() const;
    uint8_t getRcvState() const;

    bool hasLastCapture() const { return _lastCapture.hasData; }
    const IRCaptureInfo& getLastCapture() const { return _lastCapture; }
    void clearLastCapture() { _lastCapture.hasData = false; }

    static void printStructuredOutput(const decode_results& results, const String& deviceId);
    static void printJSONOutput(const decode_results& results, const String& deviceId,
                                const uint16_t* rawData = nullptr, uint16_t rawLength = 0);

private:
    static IRReceiverDriver* s_instance;
    uint8_t _pin;
    uint16_t _bufferSize;
    uint8_t _timeout;
    IRrecv _irrecv;
    decode_results _results;
    uint32_t _captureCount = 0;
    bool _initialized = false;
    IRCaptureInfo _lastCapture;
    IRTransmitterDriver* _transmitter = nullptr;
    uint32_t _blankingWindowMs = 300;
    uint32_t _lastTxBlankingMs = 0;
};

} // namespace ac::ir
