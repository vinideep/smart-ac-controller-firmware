#pragma once
#include <Arduino.h>
#include <DHT.h>
#include "../config/config.h"
#include "../config/pins.h"

struct DHTReading {
    float temperature = 0.0f;
    float humidity = 0.0f;
    float temperature_c = 0.0f;
    float humidity_percent = 0.0f;
    uint32_t timestamp_ms = 0;
    const char* status = "not_ready";
    bool valid = false;
    const char* error_message = nullptr;

    bool isStale(uint32_t maxAgeMs = 5000) const {
        if (!valid || timestamp_ms == 0) return true;
        return (millis() - timestamp_ms) > maxAgeMs;
    }
};

class DHTDriver {
public:
    explicit DHTDriver(uint8_t pin = DHT_PIN, uint8_t type = DHT_TYPE);

    void begin();
    bool update(); // Non-blocking poll; returns true when a fresh sample is processed
    const DHTReading& getLatestReading() const { return _latestReading; }
    
    uint32_t getSuccessfulReadCount() const { return _successfulReads; }
    uint32_t getErrorCount() const { return _totalErrors; }
    uint8_t getConsecutiveErrors() const { return _consecutiveErrors; }
    uint8_t getPin() const { return _pin; }

private:
    uint8_t _pin;
    uint8_t _type;
    DHT _dht;
    DHTReading _latestReading;
    uint32_t _lastReadTime = 0;
    uint32_t _successfulReads = 0;
    uint32_t _totalErrors = 0;
    uint8_t _consecutiveErrors = 0;
    bool _initialized = false;

    bool validateReading(float temp, float hum, const char*& errorReason);
};
