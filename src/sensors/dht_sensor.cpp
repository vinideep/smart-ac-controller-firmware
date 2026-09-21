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

        computePsychrometrics(temp, hum, now);
    } else {
        _consecutiveErrors++;
        _totalErrors++;
        _latestReading.temperature = NAN;
        _latestReading.humidity = NAN;
        _latestReading.temperature_c = NAN;
        _latestReading.humidity_percent = NAN;
        _latestReading.heat_index_c = NAN;
        _latestReading.dew_point_c = NAN;
        _latestReading.vpd_kpa = NAN;
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

void DHTDriver::computePsychrometrics(float temp, float hum, uint32_t now) {
    // 1. Dew Point (°C) using Magnus-Tetens formula
    const float a = 17.27f;
    const float b = 237.7f;
    float alpha = ((a * temp) / (b + temp)) + logf(hum / 100.0f);
    _latestReading.dew_point_c = (b * alpha) / (a - alpha);

    // 2. Heat Index / "Feels Like" (°C) using Rothfusz regression equation
    if (temp < 27.0f) {
        _latestReading.heat_index_c = temp;
    } else {
        float tempF = temp * 1.8f + 32.0f;
        float hiF = 0.5f * (tempF + 61.0f + ((tempF - 68.0f) * 1.2f) + (hum * 0.094f));
        if (hiF >= 80.0f) {
            hiF = -42.379f + 2.04901523f * tempF + 10.14333127f * hum
                - 0.22475541f * tempF * hum - 0.00683783f * tempF * tempF
                - 0.05481717f * hum * hum + 0.00122874f * tempF * tempF * hum
                + 0.00085282f * tempF * hum * hum - 0.00000199f * tempF * tempF * hum * hum;
        }
        _latestReading.heat_index_c = (hiF - 32.0f) / 1.8f;
    }

    // 3. Vapor Pressure Deficit (VPD in kPa)
    float vpSat = 0.61078f * expf((17.27f * temp) / (temp + 237.3f));
    _latestReading.vpd_kpa = vpSat * (1.0f - (hum / 100.0f));

    // 4. Mold Risk Score (0 - 100%) and Level
    float moldScore = 0.0f;
    if (hum > 55.0f && temp >= 18.0f && temp <= 38.0f) {
        if (hum <= 70.0f) {
            moldScore = (hum - 55.0f) * 2.0f; // 0 to 30%
        } else if (hum <= 80.0f) {
            moldScore = 30.0f + (hum - 70.0f) * 4.0f; // 30 to 70%
        } else {
            moldScore = 70.0f + (hum - 80.0f) * 1.5f; // 70 to 100%
        }
    }
    if (moldScore > 100.0f) moldScore = 100.0f;
    _latestReading.mold_risk_score = moldScore;
    if (moldScore < 25.0f) _latestReading.mold_risk_level = "Safe";
    else if (moldScore < 60.0f) _latestReading.mold_risk_level = "Moderate";
    else _latestReading.mold_risk_level = "High Risk";

    // 5. Thermal Comfort Category
    if (temp < 20.0f) {
        _latestReading.comfort_status = "Cool";
    } else if (temp > 29.0f || _latestReading.heat_index_c > 32.0f) {
        _latestReading.comfort_status = "Hot";
    } else if (hum > 68.0f) {
        _latestReading.comfort_status = "Humid";
    } else if (temp > 26.5f) {
        _latestReading.comfort_status = "Warm";
    } else {
        _latestReading.comfort_status = "Comfortable";
    }

    // 6. Rolling Room Thermal Dynamics (°C / hour)
    _historyTemps[_historyIdx] = temp;
    _historyTimes[_historyIdx] = now;
    _historyIdx = (_historyIdx + 1) % 10;
    if (_historyCount < 10) _historyCount++;

    if (_historyCount >= 3) {
        uint8_t oldestIdx = (_historyIdx + 10 - _historyCount) % 10;
        float dtSeconds = (now - _historyTimes[oldestIdx]) / 1000.0f;
        if (dtSeconds >= 4.0f) {
            float dTemp = temp - _historyTemps[oldestIdx];
            _latestReading.thermal_rate_c_per_hr = (dTemp / dtSeconds) * 3600.0f;
        }
    }
}
