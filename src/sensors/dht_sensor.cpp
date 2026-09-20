#include "dht_sensor.h"
#include <math.h>

DHTDriver::DHTDriver(uint8_t pin, uint8_t type)
    : _pin(pin), _type(type), _dht(pin, type) {}

void DHTDriver::begin() {
    _dht.begin();
    _lastReadTime = millis();
    _initialized = true;
}

bool DHTDriver::validateReading(float temp, float hum, const char*& errorReason) {
    if (isnan(temp) || isnan(hum)) {
        errorReason = "NaN received from DHT sensor (timeout or no response)";
        return false;
    }
    if (temp < DHT_TEMP_MIN_C || temp > DHT_TEMP_MAX_C) {
        errorReason = "Temperature reading out of operational range (-40C to 80C)";
        return false;
    }
    if (hum < DHT_HUMIDITY_MIN || hum > DHT_HUMIDITY_MAX) {
        errorReason = "Humidity reading out of operational range (0% to 100%)";
        return false;
    }
    return true;
}

bool DHTDriver::update() {
    if (!_initialized) {
        begin();
    }

    uint32_t now = millis();
    if (now - _lastReadTime < DHT_SAMPLE_INTERVAL_MS) {
        return false; // Not yet time to sample
    }

    _lastReadTime = now;

    // Read values from DHT sensor
    float hum = _dht.readHumidity();
    float temp = _dht.readTemperature();

    const char* errorReason = nullptr;
    if (validateReading(temp, hum, errorReason)) {
        _latestReading.temperature = temp;
        _latestReading.humidity = hum;
        _latestReading.temperature_c = temp;
        _latestReading.humidity_percent = hum;
        _latestReading.timestamp_ms = now;
        _latestReading.status = "ok";
        _latestReading.valid = true;
        _latestReading.error_message = nullptr;
        _consecutiveErrors = 0;
        _successfulReads++;
    } else {
        _consecutiveErrors++;
        _totalErrors++;
        _latestReading.temperature = NAN;
        _latestReading.humidity = NAN;
        _latestReading.temperature_c = NAN;
        _latestReading.humidity_percent = NAN;
        _latestReading.timestamp_ms = now;
        _latestReading.valid = false;
        _latestReading.error_message = errorReason;
        if (_consecutiveErrors >= DHT_MAX_CONSECUTIVE_ERRORS) {
            _latestReading.status = "sensor_disconnected";
        } else {
            _latestReading.status = "read_failed";
        }
    }

    return true;
}
