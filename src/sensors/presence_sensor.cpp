#include "presence_sensor.h"
#include "../storage/storage_manager.h"
#include <HardwareSerial.h>

namespace ac::sensors {

PresenceSensorDriver::PresenceSensorDriver(uint8_t pin, bool activeLow, int8_t rxPin, int8_t txPin, uint32_t baud)
    : _pin(pin), _activeLow(activeLow), _rxPin(rxPin), _txPin(txPin), _baud(baud) {}

void PresenceSensorDriver::begin() {
    if (!_initialized) {
        pinMode(_pin, _activeLow ? INPUT_PULLUP : INPUT_PULLDOWN);
        if (_rxPin >= 0 && _txPin >= 0) {
            Serial2.begin(_baud, SERIAL_8N1, _rxPin, _txPin);
        }
        storage::RadarStorageConfig cfg;
        if (storage::StorageManager::loadRadarConfig(cfg)) {
            _minDistanceM = cfg.minDistanceM;
            _maxDistanceM = cfg.maxDistanceM;
            _minMovingEnergy = cfg.minMovingEnergy;
            _minStationaryEnergy = cfg.minStationaryEnergy;
            _absenceTimeoutMs = cfg.absenceTimeoutS * 1000;
        }
        _stateStartTime = millis();
        _reading.last_seen_ms = millis();
        _lastPresenceDetectedMs = millis();
        _initialized = true;
    }
}

void PresenceSensorDriver::setActiveLow(bool activeLow) {
    _activeLow = activeLow;
    if (_initialized) {
        pinMode(_pin, _activeLow ? INPUT_PULLUP : INPUT_PULLDOWN);
    }
}

bool PresenceSensorDriver::parseFrame(const uint8_t* buf, size_t len) {
    if (len < 16) return false;
    // Check Header 0xF4 0xF3 0xF2 0xF1
    if (buf[0] != 0xF4 || buf[1] != 0xF3 || buf[2] != 0xF2 || buf[3] != 0xF1) {
        return false;
    }

    uint8_t targetState = buf[7];
    uint16_t moveDist = buf[8] | (buf[9] << 8);
    uint8_t moveEnergy = buf[10];
    uint16_t statDist = buf[11] | (buf[12] << 8);
    uint8_t statEnergy = buf[13];
    uint16_t detectDist = buf[14] | (buf[15] << 8);

    _reading.moving_distance_cm = moveDist;
    _reading.moving_energy = moveEnergy;
    _reading.stationary_distance_cm = statDist;
    _reading.stationary_energy = statEnergy;

    float moveDistM = moveDist / 100.0f;
    float statDistM = statDist / 100.0f;

    bool moveValid = (targetState == 0x01 || targetState == 0x03) &&
                     (moveDistM >= _minDistanceM && moveDistM <= _maxDistanceM) &&
                     (moveEnergy >= _minMovingEnergy);

    bool statValid = (targetState == 0x02 || targetState == 0x03) &&
                     (statDistM >= _minDistanceM && statDistM <= _maxDistanceM) &&
                     (statEnergy >= _minStationaryEnergy);

    if (moveValid && statValid) {
        _rawInZoneDetected = true;
        _reading.target_count = 2;
        _reading.motion_state = "Moving + Stationary (In Zone)";
        _reading.distance_m = (moveDistM > 0.05f && moveDistM < statDistM) ? moveDistM : statDistM;
    } else if (moveValid) {
        _rawInZoneDetected = true;
        _reading.target_count = 1;
        _reading.motion_state = "Moving Target (In Zone)";
        _reading.distance_m = moveDistM;
    } else if (statValid) {
        _rawInZoneDetected = true;
        _reading.target_count = 1;
        _reading.motion_state = "Stationary / Human Present (In Zone)";
        _reading.distance_m = statDistM;
    } else {
        _rawInZoneDetected = false;
        _reading.target_count = 0;
        _reading.motion_state = (targetState != 0x00) ? "Filtered Out-of-Zone" : "None";
        _reading.distance_m = 0.0f;
    }

    return true;
}

void PresenceSensorDriver::processUart() {
    if (_rxPin < 0) return;

    while (Serial2.available() > 0) {
        uint8_t byte = Serial2.read();
        _totalUartBytes++;
        if (_rxLen < sizeof(_rxBuf)) {
            _rxBuf[_rxLen++] = byte;
        } else {
            // Buffer overflow, shift left
            memmove(_rxBuf, _rxBuf + 1, sizeof(_rxBuf) - 1);
            _rxBuf[sizeof(_rxBuf) - 1] = byte;
        }

        // Look for header 0xF4 0xF3 0xF2 0xF1
        if (_rxLen >= 10) {
            for (size_t i = 0; i + 4 <= _rxLen; i++) {
                if (_rxBuf[i] == 0xF4 && _rxBuf[i + 1] == 0xF3 && _rxBuf[i + 2] == 0xF2 && _rxBuf[i + 3] == 0xF1) {
                    if (i > 0) {
                        memmove(_rxBuf, _rxBuf + i, _rxLen - i);
                        _rxLen -= i;
                    }
                    if (_rxLen >= 6) {
                        uint16_t payloadLen = _rxBuf[4] | (_rxBuf[5] << 8);
                        size_t totalFrameLen = 4 + 2 + payloadLen + 4; // Header(4) + Len(2) + Payload(payloadLen) + Tail(4)
                        if (totalFrameLen <= sizeof(_rxBuf) && _rxLen >= totalFrameLen) {
                            // Check tail 0xF8 0xF7 0xF6 0xF5
                            size_t tailIdx = totalFrameLen - 4;
                            if (_rxBuf[tailIdx] == 0xF8 && _rxBuf[tailIdx + 1] == 0xF7 &&
                                _rxBuf[tailIdx + 2] == 0xF6 && _rxBuf[tailIdx + 3] == 0xF5) {
                                if (parseFrame(_rxBuf, totalFrameLen)) {
                                    _lastUartFrameTime = millis();
                                }
                            }
                            memmove(_rxBuf, _rxBuf + totalFrameLen, _rxLen - totalFrameLen);
                            _rxLen -= totalFrameLen;
                        }
                    }
                    break;
                }
            }
        }
    }
}

bool PresenceSensorDriver::update() {
    if (!_initialized) begin();

    uint32_t now = millis();
    if (now - _lastPollTime < 50) {
        return false;
    }
    _lastPollTime = now;

    processUart();

    bool uartActive = (_lastUartFrameTime > 0 && (now - _lastUartFrameTime < 2500));

    if (uartActive) {
        if (_rawInZoneDetected) {
            _lastPresenceDetectedMs = now;
            _reading.present = true;
        } else {
            // Asymmetric Debounce: hold active presence until absence timeout expires
            if (_lastPresenceDetectedMs == 0 || (now - _lastPresenceDetectedMs >= _absenceTimeoutMs)) {
                _reading.present = false;
            } else {
                _reading.present = true;
            }
        }
    } else {
        int pinLevel = digitalRead(_pin);
        bool detected = _activeLow ? (pinLevel == LOW) : (pinLevel == HIGH);

        if (detected == _rawState) {
            if (_debounceCount < 3) {
                _debounceCount++;
            }
        } else {
            _rawState = detected;
            _debounceCount = 0;
        }

        if (_debounceCount >= 3) {
            if (detected) {
                _lastPresenceDetectedMs = now;
                _reading.present = true;
                _reading.motion_state = "Micro-motion / Human Present";
                _reading.target_count = 1;
                _reading.distance_m = 1.8f;
            } else {
                if (_lastPresenceDetectedMs == 0 || (now - _lastPresenceDetectedMs >= _absenceTimeoutMs)) {
                    _reading.present = false;
                    _reading.motion_state = "None";
                    _reading.target_count = 0;
                    _reading.distance_m = 0.0f;
                } else {
                    _reading.present = true;
                }
            }
        }
    }

    bool stateChanged = false;
    static bool prevReportedState = false;

    if (_reading.present != prevReportedState) {
        prevReportedState = _reading.present;
        _stateStartTime = now;
        stateChanged = true;
    }

    _reading.duration_seconds = (now - _stateStartTime) / 1000;
    if (_reading.present) {
        _reading.last_seen_ms = now;
        _reading.status = "ROOM_OCCUPIED";
    } else {
        _reading.status = "ROOM_EMPTY";
    }

    return stateChanged;
}

void PresenceSensorDriver::setDistanceGates(float minM, float maxM) {
    if (minM >= 0.0f && maxM > minM && maxM <= 10.0f) {
        _minDistanceM = minM;
        _maxDistanceM = maxM;
        storage::RadarStorageConfig cfg;
        cfg.minDistanceM = _minDistanceM;
        cfg.maxDistanceM = _maxDistanceM;
        cfg.minMovingEnergy = _minMovingEnergy;
        cfg.minStationaryEnergy = _minStationaryEnergy;
        cfg.absenceTimeoutS = _absenceTimeoutMs / 1000;
        storage::StorageManager::saveRadarConfig(cfg);
        Serial.printf("[RADAR] Distance gates updated: %.2fm to %.2fm\n", _minDistanceM, _maxDistanceM);
    }
}

void PresenceSensorDriver::setEnergyThresholds(uint8_t minMoveEnergy, uint8_t minStatEnergy) {
    _minMovingEnergy = minMoveEnergy;
    _minStationaryEnergy = minStatEnergy;
    storage::RadarStorageConfig cfg;
    cfg.minDistanceM = _minDistanceM;
    cfg.maxDistanceM = _maxDistanceM;
    cfg.minMovingEnergy = _minMovingEnergy;
    cfg.minStationaryEnergy = _minStationaryEnergy;
    cfg.absenceTimeoutS = _absenceTimeoutMs / 1000;
    storage::StorageManager::saveRadarConfig(cfg);
    Serial.printf("[RADAR] Energy thresholds updated: Move=%u%%, Stat=%u%%\n", _minMovingEnergy, _minStationaryEnergy);
}

void PresenceSensorDriver::setAbsenceTimeoutSeconds(uint32_t seconds) {
    if (seconds >= 5 && seconds <= 3600) {
        _absenceTimeoutMs = seconds * 1000;
        storage::RadarStorageConfig cfg;
        cfg.minDistanceM = _minDistanceM;
        cfg.maxDistanceM = _maxDistanceM;
        cfg.minMovingEnergy = _minMovingEnergy;
        cfg.minStationaryEnergy = _minStationaryEnergy;
        cfg.absenceTimeoutS = seconds;
        storage::StorageManager::saveRadarConfig(cfg);
        Serial.printf("[RADAR] Absence debounce timeout updated: %u seconds\n", seconds);
    }
}

void PresenceSensorDriver::toJSON(JsonDocument& doc) const {
    doc["presence"] = _reading.present;
    doc["duration_seconds"] = _reading.duration_seconds;
    doc["last_seen_ms"] = _reading.last_seen_ms;
    doc["status"] = _reading.status;
    doc["distance_m"] = _reading.distance_m;
    doc["target_count"] = _reading.target_count;
    doc["motion_state"] = _reading.motion_state;
    doc["moving_energy"] = _reading.moving_energy;
    doc["stationary_energy"] = _reading.stationary_energy;
    doc["min_distance_m"] = _minDistanceM;
    doc["max_distance_m"] = _maxDistanceM;
    doc["min_moving_energy"] = _minMovingEnergy;
    doc["min_stationary_energy"] = _minStationaryEnergy;
    doc["absence_timeout_s"] = _absenceTimeoutMs / 1000;
    doc["in_zone"] = _rawInZoneDetected;
    doc["uart_active"] = isUartActive();
    doc["uart_bytes_received"] = _totalUartBytes;
}

void PresenceSensorDriver::printDebug(Print& out) {
    out.printf("[RADAR_DBG] totalBytes=%u, rxLen=%u, uartActive=%d, lastFrameAgo=%ums\n",
               _totalUartBytes, _rxLen, isUartActive(), _lastUartFrameTime > 0 ? (millis() - _lastUartFrameTime) : 0);
    out.print("[RADAR_DBG] buffer: ");
    for (size_t k = 0; k < _rxLen && k < 32; k++) {
        out.printf("%02X ", _rxBuf[k]);
    }
    out.println();
}

} // namespace ac::sensors
