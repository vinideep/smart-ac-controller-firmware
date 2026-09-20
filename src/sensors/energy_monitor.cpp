#include "energy_monitor.h"
#include <math.h>

namespace ac::sensors {

EnergyMonitor::EnergyMonitor(float defaultTariff, float nominalVoltage)
    : _nominalVoltage(nominalVoltage) {
    _reading.voltage = nominalVoltage;
    _reading.current = 0.015f;
    _reading.power_watts = 2.5f;
    _reading.power_factor = 0.65f;
    _reading.energy_kwh_today = 0.0f;
    _reading.estimated_cost_today = 0.0f;
    _reading.tariff_rate = defaultTariff > 0.0f ? defaultTariff : 8.0f;
    _reading.timestamp_ms = 0;
    _reading.status = "standby";
    _reading.valid = true;
}

void EnergyMonitor::begin() {
    _lastUpdateTime = millis();
    _reading.timestamp_ms = _lastUpdateTime;
    _initialized = true;
}

bool EnergyMonitor::setTariff(float ratePerKwh) {
    if (ratePerKwh <= 0.0f) {
        return false;
    }
    _reading.tariff_rate = ratePerKwh;
    _reading.estimated_cost_today = static_cast<float>(_accumulatedKwh * _reading.tariff_rate);
    return true;
}

void EnergyMonitor::setNominalVoltage(float volts) {
    if (volts > 50.0f && volts < 300.0f) {
        _nominalVoltage = volts;
        _reading.voltage = volts;
    }
}

void EnergyMonitor::resetDaily() {
    _accumulatedKwh = 0.0;
    _reading.energy_kwh_today = 0.0f;
    _reading.estimated_cost_today = 0.0f;
}

void EnergyMonitor::accumulateEnergy(uint32_t nowMs) {
    if (_lastUpdateTime == 0) {
        _lastUpdateTime = nowMs;
        _reading.timestamp_ms = nowMs;
        return;
    }

    uint32_t deltaMs = nowMs - _lastUpdateTime;
    if (deltaMs == 0) {
        return;
    }

    // Protection against corrupt jumps (> 24 hours)
    if (deltaMs > 86400000UL) {
        _lastUpdateTime = nowMs;
        _reading.timestamp_ms = nowMs;
        return;
    }

    double deltaHours = static_cast<double>(deltaMs) / 3600000.0;
    double deltaKwh = (static_cast<double>(_reading.power_watts) * deltaHours) / 1000.0;
    _accumulatedKwh += deltaKwh;
    _reading.energy_kwh_today = static_cast<float>(_accumulatedKwh);
    _reading.estimated_cost_today = static_cast<float>(_accumulatedKwh * _reading.tariff_rate);
    _reading.timestamp_ms = nowMs;
    _lastUpdateTime = nowMs;
}


bool EnergyMonitor::update(uint32_t nowMs) {
    if (!_initialized) {
        begin();
    }
    uint32_t current = (nowMs == 0) ? millis() : nowMs;
    accumulateEnergy(current);
    return true;
}

void EnergyMonitor::updateMeasurement(float voltage, float current, float powerWatts, float powerFactor, const char* status, uint32_t nowMs) {
    uint32_t currentMs = (nowMs == 0) ? millis() : nowMs;
    accumulateEnergy(currentMs);

    _reading.voltage = voltage;
    _reading.current = current;
    _reading.power_watts = powerWatts;
    _reading.power_factor = powerFactor;
    _reading.status = status ? status : "measuring";
    _reading.valid = true;
    _reading.timestamp_ms = currentMs;
}

void EnergyMonitor::updateFromAcState(bool isPowered,
                                     const String& mode,
                                     const String& fanSpeed,
                                     uint8_t targetTemp,
                                     float ambientTemp,
                                     uint32_t nowMs) {
    uint32_t currentMs = (nowMs == 0) ? millis() : nowMs;
    accumulateEnergy(currentMs);

    _reading.voltage = _nominalVoltage;
    _reading.timestamp_ms = currentMs;
    _reading.valid = true;

    if (!isPowered) {
        // Standby idle electronics draw
        _reading.power_watts = 2.5f;
        _reading.power_factor = 0.65f;
        _reading.current = _reading.power_watts / (_reading.voltage * _reading.power_factor);
        _reading.status = "standby";
        return;
    }

    // Determine evaporator fan power contribution
    float fanPower = 45.0f; // Default Auto
    if (fanSpeed.equalsIgnoreCase("low")) {
        fanPower = 35.0f;
    } else if (fanSpeed.equalsIgnoreCase("med")) {
        fanPower = 55.0f;
    } else if (fanSpeed.equalsIgnoreCase("high")) {
        fanPower = 80.0f;
    }

    if (mode.equalsIgnoreCase("fan")) {
        _reading.power_watts = fanPower;
        _reading.power_factor = 0.88f;
        _reading.current = _reading.power_watts / (_reading.voltage * _reading.power_factor);
        _reading.status = "fan_only";
        return;
    }

    if (mode.equalsIgnoreCase("dry")) {
        _reading.power_watts = 600.0f + fanPower;
        _reading.power_factor = 0.94f;
        _reading.current = _reading.power_watts / (_reading.voltage * _reading.power_factor);
        _reading.status = "dehumidifying";
        return;
    }


    // Cool or Auto mode: Inverter compressor running
    _reading.status = "cooling";
    _reading.power_factor = 0.97f;

    float deltaTemp = ambientTemp - static_cast<float>(targetTemp);
    float compressorPower = 1100.0f;

    if (deltaTemp > 0.0f) {
        // Moderate to heavy cooling load: modulate compressor up
        compressorPower = 1100.0f + (deltaTemp * 85.0f);
    } else {
        // Room reached target or below: inverter compressor modulates down
        compressorPower = 550.0f;
    }

    // Operational limits of 1.5-ton inverter unit: 500W min - 1800W max
    if (compressorPower < 500.0f) compressorPower = 500.0f;
    if (compressorPower > 1800.0f) compressorPower = 1800.0f;

    _reading.power_watts = compressorPower + fanPower;
    _reading.current = _reading.power_watts / (_reading.voltage * _reading.power_factor);
}

void EnergyMonitor::toJSON(JsonDocument& doc) const {
    doc["voltage"] = round(_reading.voltage * 10.0f) / 10.0f;
    doc["current"] = round(_reading.current * 100.0f) / 100.0f;
    doc["power_watts"] = round(_reading.power_watts * 10.0f) / 10.0f;
    doc["power_factor"] = round(_reading.power_factor * 100.0f) / 100.0f;
    doc["energy_kwh_today"] = round(_reading.energy_kwh_today * 10000.0f) / 10000.0f;
    doc["tariff_rate"] = round(_reading.tariff_rate * 100.0f) / 100.0f;
    doc["estimated_cost_today"] = round(_reading.estimated_cost_today * 100.0f) / 100.0f;
    doc["status"] = _reading.status;
    doc["timestamp_ms"] = _reading.timestamp_ms;
    doc["valid"] = _reading.valid;
}

String EnergyMonitor::toJSONString() const {
    JsonDocument doc;
    toJSON(doc);
    String out;
    serializeJson(doc, out);
    return out;
}

} // namespace ac::sensors
