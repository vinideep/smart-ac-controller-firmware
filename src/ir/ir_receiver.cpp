#include "ir_receiver.h"
#include "ir_transmitter.h"
#include "ir_protocol_azure.h"
#include <ArduinoJson.h>

namespace _IRrecv {
extern atomic_irparams_t params;
}

namespace ac::ir {

IRReceiverDriver* IRReceiverDriver::s_instance = nullptr;

IRReceiverDriver::IRReceiverDriver(uint8_t pin, uint16_t bufferSize, uint8_t timeout)
    : _pin(pin),
      _bufferSize(bufferSize),
      _timeout(timeout),
      _irrecv(pin, bufferSize, timeout, true) {
    s_instance = this;
}

void IRReceiverDriver::begin() {
    _irrecv.setUnknownThreshold(kMinNoiseThreshold);
    _irrecv.enableIRIn(true); // Enable with internal pull-up
    _initialized = true;
}

uint16_t IRReceiverDriver::getRawLen() const {
    return _IRrecv::params.rawlen;
}

uint8_t IRReceiverDriver::getRcvState() const {
    return _IRrecv::params.rcvstate;
}

bool IRReceiverDriver::update(const String& deviceId) {
    if (!_initialized) {
        begin();
    }

    // Software timeout fallback in case ESP32 hardware timer alarm missed
    static uint16_t lastTrackedRawlen = 0;
    static uint32_t lastPulseTime = 0;
    uint16_t curRawlen = _IRrecv::params.rawlen;
    if (curRawlen != lastTrackedRawlen) {
        lastTrackedRawlen = curRawlen;
        lastPulseTime = millis();
    } else if (curRawlen > 0 && (millis() - lastPulseTime > _timeout) && _IRrecv::params.rcvstate != kStopState) {
        _IRrecv::params.rcvstate = kStopState;
    }

    if (_irrecv.decode(&_results)) {
        if (_results.rawlen >= kMinNoiseThreshold) {
            uint32_t now = millis();
            bool isBlanked = false;
            uint32_t txDelta = 0;
            if (_transmitter) {
                if (_transmitter->isTransmitting()) {
                    isBlanked = true;
                } else if (_transmitter->getLastTxEndTime() > 0) {
                    txDelta = now - _transmitter->getLastTxEndTime();
                    if (txDelta < _blankingWindowMs) {
                        isBlanked = true;
                    }
                }
            } else if (_lastTxBlankingMs > 0) {
                txDelta = now - _lastTxBlankingMs;
                if (txDelta < _blankingWindowMs) {
                    isBlanked = true;
                }
            }

            if (isBlanked) {
                Serial.printf("[IR_BLANKING] Loopback signal suppressed during TX blanking window (%lu ms since TX)\n",
                              (unsigned long)txDelta);
                _irrecv.resume();
                return false;
            }

            _captureCount++;

            digitalWrite(STATUS_LED_PIN, HIGH);

            _lastCapture.hasData = true;
            _lastCapture.protocol = typeToString(_results.decode_type, _results.repeat);
            _lastCapture.protocolNum = (int16_t)_results.decode_type;
            _lastCapture.bits = _results.bits;
            _lastCapture.hexCode = resultToHexidecimal(&_results);
            _lastCapture.timestampMs = millis();
            _lastCapture.isAc = hasACState(_results.decode_type);
            _lastCapture.hasAcState = false;

            // Attempt to decode Azure Essence frame from raw timing transitions
            std::unique_ptr<uint16_t[]> rawPtr(resultToRawArray(&_results));
            uint16_t rawLen = getCorrectedRawLength(&_results);
            _lastCapture.rawLength = (rawLen > 200) ? 200 : rawLen;
            if (rawPtr) {
                for (uint16_t i = 0; i < _lastCapture.rawLength; i++) {
                    _lastCapture.rawData[i] = rawPtr[i];
                }
            }

            if (rawPtr && rawLen >= 146) {
                if (AzureEssenceProtocol::decodeRaw(rawPtr.get(), rawLen, _lastCapture.acState)) {
                    _lastCapture.hasAcState = true;
                    _lastCapture.isAc = true;
                    _lastCapture.protocol = "AZURE_ESSENCE";
                    Serial.printf("[IR_SNOOPER] Decoded Azure Essence remote command: Power=%s Temp=%dC Mode=%s Fan=%s\n",
                                  _lastCapture.acState.power ? "ON" : "OFF",
                                  _lastCapture.acState.temperature,
                                  _lastCapture.acState.mode.c_str(),
                                  _lastCapture.acState.fanSpeed.c_str());
                }
            }

            // Universal AC remote decoding fallback for all supported brands (Gree, Daikin, Midea, etc.)
            if (!_lastCapture.hasAcState && hasACState(_results.decode_type)) {
                stdAc::state_t stdState;
                if (IRAcUtils::decodeToState(&_results, &stdState)) {
                    _lastCapture.hasAcState = true;
                    _lastCapture.isAc = true;
                    _lastCapture.acState.power = stdState.power;
                    _lastCapture.acState.temperature = (uint8_t)round(stdState.degrees);
                    if (_lastCapture.acState.temperature < 16) _lastCapture.acState.temperature = 16;
                    if (_lastCapture.acState.temperature > 31) _lastCapture.acState.temperature = 31;

                    switch (stdState.mode) {
                        case stdAc::opmode_t::kCool: _lastCapture.acState.mode = "cool"; break;
                        case stdAc::opmode_t::kHeat: _lastCapture.acState.mode = "heat"; break;
                        case stdAc::opmode_t::kDry:  _lastCapture.acState.mode = "dry"; break;
                        case stdAc::opmode_t::kFan:  _lastCapture.acState.mode = "fan"; break;
                        case stdAc::opmode_t::kAuto: _lastCapture.acState.mode = "auto"; break;
                        default: _lastCapture.acState.mode = "cool"; break;
                    }

                    switch (stdState.fanspeed) {
                        case stdAc::fanspeed_t::kLow:
                        case stdAc::fanspeed_t::kMin:
                            _lastCapture.acState.fanSpeed = "low"; break;
                        case stdAc::fanspeed_t::kMedium:
                            _lastCapture.acState.fanSpeed = "med"; break;
                        case stdAc::fanspeed_t::kHigh:
                        case stdAc::fanspeed_t::kMax:
                            _lastCapture.acState.fanSpeed = "high"; break;
                        default:
                            _lastCapture.acState.fanSpeed = "auto"; break;
                    }

                    _lastCapture.acState.swing = (stdState.swingv != stdAc::swingv_t::kOff || stdState.swingh != stdAc::swingh_t::kOff);
                    _lastCapture.acState.sleep = (stdState.sleep >= 0);
                    _lastCapture.acState.source = "ir_remote";
                    _lastCapture.acState.timestamp = millis();

                    Serial.printf("[IR_MIRROR] Universal AC State Decoded (%s): Power=%s Temp=%dC Mode=%s Fan=%s\n",
                                  _lastCapture.protocol.c_str(),
                                  _lastCapture.acState.power ? "ON" : "OFF",
                                  _lastCapture.acState.temperature,
                                  _lastCapture.acState.mode.c_str(),
                                  _lastCapture.acState.fanSpeed.c_str());
                }
            }

            printStructuredOutput(_results, deviceId);

            digitalWrite(STATUS_LED_PIN, LOW);
            _irrecv.resume();
            return true;
        }
        _irrecv.resume();
    }

    return false;
}

void IRReceiverDriver::pause() {
    _irrecv.pause();
}

void IRReceiverDriver::resume() {
    _irrecv.resume();
}

void IRReceiverDriver::enable() {
    _irrecv.enableIRIn(true);
}

void IRReceiverDriver::disable() {
    _irrecv.disableIRIn();
}

void IRReceiverDriver::printStructuredOutput(const decode_results& results, const String& deviceId) {
    String protocolStr = typeToString(results.decode_type, results.repeat);
    String hexStr = resultToHexidecimal(&results);
    bool isAc = hasACState(results.decode_type);
    String acDesc = IRAcUtils::resultAcToString(&results);
    uint16_t rawLength = getCorrectedRawLength(&results);
    std::unique_ptr<uint16_t[]> rawPtr(resultToRawArray(&results));
    uint16_t* rawData = rawPtr.get();
    uint16_t nbytes = (results.bits + 7) / 8;

    Serial.println();
    Serial.println("==================================================");
    Serial.println("              IR SIGNAL CAPTURED                  ");
    Serial.println("==================================================");
    Serial.printf("[CAPTURE] Device ID    : %s\n", deviceId.c_str());
    Serial.printf("[CAPTURE] Timestamp    : %u ms\n", millis());
    Serial.printf("[CAPTURE] Protocol     : %s (Type %d)\n", protocolStr.c_str(), (int)results.decode_type);
    Serial.printf("[CAPTURE] Bit Length   : %u bits\n", results.bits);
    Serial.printf("[CAPTURE] Code / Hex   : %s\n", hexStr.c_str());
    Serial.printf("[CAPTURE] Repeat Flag  : %s\n", results.repeat ? "true" : "false");
    if (results.overflow) {
        Serial.println("[WARNING] Capture Buffer Overflow! Pulse train exceeded buffer capacity.");
    }
    if (acDesc.length() > 0) {
        Serial.printf("[CAPTURE] Decoded AC   : %s\n", acDesc.c_str());
    }
    if (isAc && nbytes > 0) {
        Serial.print("[CAPTURE] State Bytes  : ");
        for (uint16_t i = 0; i < nbytes; i++) {
            Serial.printf("%02X ", results.state[i]);
        }
        Serial.println();
    }
    Serial.printf("[CAPTURE] Raw Length   : %u transitions\n", rawLength);
    Serial.println("[CAPTURE] Raw Timing Array (microseconds):");
    Serial.print("[");
    if (rawData != nullptr) {
        for (uint16_t i = 0; i < rawLength; i++) {
            Serial.print(rawData[i]);
            if (i + 1 < rawLength) {
                Serial.print(", ");
                if ((i + 1) % 12 == 0) {
                    Serial.println();
                    Serial.print(" ");
                }
            }
        }
    }
    Serial.println("]");
    Serial.println("--------------------------------------------------");
    Serial.println("[CAPTURE] JSON Telemetry Format:");
    printJSONOutput(results, deviceId, rawData, rawLength);
    Serial.println("==================================================");
    Serial.println();
}

void IRReceiverDriver::printJSONOutput(const decode_results& results, const String& deviceId,
                                      const uint16_t* rawData, uint16_t rawLength) {
    std::unique_ptr<uint16_t[]> fallbackRawPtr;
    if (rawData == nullptr) {
        rawLength = getCorrectedRawLength(&results);
        fallbackRawPtr.reset(resultToRawArray(&results));
        rawData = fallbackRawPtr.get();
    }

    JsonDocument doc;

    doc["type"] = "ir_capture";
    doc["device_id"] = deviceId;
    doc["timestamp_ms"] = millis();
    doc["protocol"] = typeToString(results.decode_type, results.repeat);
    doc["protocol_num"] = (int)results.decode_type;
    doc["bits"] = results.bits;
    doc["hex"] = resultToHexidecimal(&results);
    doc["repeat"] = results.repeat;
    doc["overflow"] = results.overflow;
    doc["is_ac"] = hasACState(results.decode_type);

    String acDesc = IRAcUtils::resultAcToString(&results);
    if (acDesc.length() > 0) {
        doc["ac_desc"] = acDesc;
    }

    uint16_t nbytes = (results.bits + 7) / 8;
    if (hasACState(results.decode_type) && nbytes > 0) {
        JsonArray stateArr = doc["state"].to<JsonArray>();
        for (uint16_t i = 0; i < nbytes; i++) {
            stateArr.add(results.state[i]);
        }
    }

    doc["raw_len"] = rawLength;
    JsonArray rawArr = doc["raw"].to<JsonArray>();
    if (rawData != nullptr) {
        for (uint16_t i = 0; i < rawLength; i++) {
            rawArr.add(rawData[i]);
        }
    }

    serializeJson(doc, Serial);
    Serial.println();
}

} // namespace ac::ir
