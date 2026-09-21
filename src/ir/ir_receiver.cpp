#include "ir_receiver.h"
#include "ir_protocol_azure.h"
#include <ArduinoJson.h>

namespace _IRrecv {
extern atomic_irparams_t params;
}

namespace ac::ir {

IRReceiverDriver::IRReceiverDriver(uint8_t pin, uint16_t bufferSize, uint8_t timeout)
    : _pin(pin),
      _bufferSize(bufferSize),
      _timeout(timeout),
      _irrecv(pin, bufferSize, timeout, true) {}

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
            _captureCount++;

            digitalWrite(STATUS_LED_PIN, HIGH);
            printStructuredOutput(_results, deviceId);

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

            digitalWrite(STATUS_LED_PIN, LOW);
            _irrecv.resume();
            return true;
        }
        _irrecv.resume();
    }

    return false;
}

void IRReceiverDriver::resume() {
    _irrecv.resume();
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
