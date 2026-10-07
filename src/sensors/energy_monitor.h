#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>

namespace ac::sensors {

/**
 * @brief Represents an instantaneous AC electrical and energy measurement.
 */
struct EnergyReading {
    float voltage = 230.0f;              // Volts AC (RMS)
    float current = 0.0f;                // Amperes (RMS)
    float power_watts = 0.0f;            // Active real power (Watts)
    float power_factor = 0.95f;          // Power factor (cos phi)
    float energy_kwh_today = 0.0f;       // Accumulated energy consumption today (kWh)
    float estimated_cost_today = 0.0f;   // Estimated energy cost today based on tariff
    float tariff_rate = 8.0f;            // Configurable electricity rate per kWh (e.g., ₹8.00/kWh)
    uint32_t timestamp_ms = 0;           // Reading timestamp
    const char* status = "standby";      // Operational power state description
    bool valid = true;                   // Telemetry validity flag
};

/**
 * @brief Isolated AC Energy Meter Driver & Estimator.
 * 
 * Tracks AC voltage, current, real power, cumulative daily energy (kWh),
 * and financial cost based on a user-configurable tariff.
 * Supports direct readings from isolated physical meters (e.g., PZEM-004T / CT)
 * or deterministic load modeling of the Azure Essence inverter compressor.
 */
class EnergyMonitor {
public:
    explicit EnergyMonitor(float defaultTariff = 8.0f, float nominalVoltage = 230.0f);

    void begin();

    /**
     * @brief Integrates energy consumption based on active power over elapsed time.
     * @param nowMs Current system timestamp in milliseconds (uses millis() if 0).
     * @return true if an integration step was performed.
     */
    bool update(uint32_t nowMs = 0);

    /**
     * @brief Updates energy model from AC operational state and ambient temperature.
     * Simulates the variable-frequency inverter compressor, evaporator fan, and standby electronics.
     */
    void updateFromAcState(bool isPowered,
                           const String& mode,
                           const String& fanSpeed,
                           uint8_t targetTemp,
                           float ambientTemp,
                           uint32_t nowMs = 0);

    /**
     * @brief Directly feed measured values from an isolated physical current transformer / energy IC.
     */
    void updateMeasurement(float voltage, float current, float powerWatts, float powerFactor = 0.95f, const char* status = "measuring", uint32_t nowMs = 0);

    /**
     * @brief Sets electricity tariff rate per kWh.
     */
    bool setTariff(float ratePerKwh);
    float getTariff() const { return _reading.tariff_rate; }

    void setNominalVoltage(float volts);
    float getNominalVoltage() const { return _nominalVoltage; }

    const EnergyReading& getReading() const { return _reading; }
    float getVoltage() const { return _reading.voltage; }
    float getCurrent() const { return _reading.current; }
    float getPowerWatts() const { return _reading.power_watts; }
    float getEnergyKwhToday() const { return _reading.energy_kwh_today; }
    float getEstimatedCostToday() const { return _reading.estimated_cost_today; }

    // Power Tier Classification
    enum class PowerStateTier {
        Standby,
        FanOnly,
        CompressorActive
    };

    PowerStateTier getPowerTier() const {
        if (_reading.power_watts >= 250.0f) return PowerStateTier::CompressorActive;
        if (_reading.power_watts >= 20.0f) return PowerStateTier::FanOnly;
        return PowerStateTier::Standby;
    }

    const char* getPowerTierStr() const {
        switch (getPowerTier()) {
            case PowerStateTier::CompressorActive: return "compressor_cooling";
            case PowerStateTier::FanOnly: return "fan_only";
            case PowerStateTier::Standby:
            default: return "standby";
        }
    }

    bool isCompressorActive() const { return _reading.power_watts >= 250.0f; }
    bool isFanActive() const { return _reading.power_watts >= 20.0f && _reading.power_watts < 250.0f; }
    bool isStandby() const { return _reading.power_watts < 20.0f; }
    bool isHardwareSensorActive() const { return _hardwareSensorActive; }

    void setThermalStallSuspended(bool suspended) {
        _thermalStallSuspended = suspended;
        if (suspended && !_hardwareSensorActive) {
            _reading.power_watts = 2.5f;
            _reading.power_factor = 0.65f;
            _reading.current = _reading.power_watts / (_reading.voltage * _reading.power_factor);
            _reading.status = "thermal_stall_desync";
        }
    }
    bool isThermalStallSuspended() const { return _thermalStallSuspended; }

    /**
     * @brief Samples physical analog current from a CT clamp or ACS712 sensor.
     * @param pin Analog ADC pin (e.g. GPIO 34)
     * @param calibration Current transformer calibration factor (e.g. 30.0 for SCT-013-030)
     * @return true if sampling succeeded
     */
    bool sampleAdcCurrent(uint8_t pin, float calibration = 30.0f);

    /**
     * @brief Resets cumulative daily energy and cost counters (e.g. at midnight).
     */
    void resetDaily();

    /**
     * @brief Serializes current energy metrics into an ArduinoJson document.
     */
    void toJSON(JsonDocument& doc) const;

    /**
     * @brief Serializes current energy metrics into a JSON string.
     */
    String toJSONString() const;

private:
    EnergyReading _reading;
    float _nominalVoltage;
    double _accumulatedKwh = 0.0;
    uint32_t _lastUpdateTime = 0;
    bool _initialized = false;
    bool _hardwareSensorActive = false;
    bool _thermalStallSuspended = false;

    void accumulateEnergy(uint32_t nowMs);
};

} // namespace ac::sensors

