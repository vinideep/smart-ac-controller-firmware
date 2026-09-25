#include "telemetry.h"
#include <esp_system.h>
#include <rom/rtc.h>
#include "../config/config.h"
#include "../config/pins.h"

static const char* getResetReasonString(RESET_REASON reason) {
    switch (reason) {
        case 1:  return "POWERON_RESET";
        case 3:  return "SW_RESET";
        case 4:  return "OWDT_RESET";
        case 5:  return "DEEPSLEEP_RESET";
        case 6:  return "SDIO_RESET";
        case 7:  return "TG0WDT_SYS_RESET";
        case 8:  return "TG1WDT_SYS_RESET";
        case 9:  return "RTCWDT_SYS_RESET";
        case 10: return "INTRUSION_RESET";
        case 11: return "TGWDT_CPU_RESET";
        case 12: return "SW_CPU_RESET";
        case 13: return "RTCWDT_CPU_RESET";
        case 14: return "EXT_CPU_RESET";
        case 15: return "RTCWDT_BROWN_OUT_RESET";
        case 16: return "RTCWDT_RTC_RESET";
        default: return "UNKNOWN_RESET";
    }
}

void TelemetryManager::printDeviceInfo(const String& deviceId) {
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    uint64_t mac = ESP.getEfuseMac();
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             (uint8_t)(mac),
             (uint8_t)(mac >> 8),
             (uint8_t)(mac >> 16),
             (uint8_t)(mac >> 24),
             (uint8_t)(mac >> 32),
             (uint8_t)(mac >> 40));

    Serial.println();
    Serial.println("==================================================");
    Serial.printf("  %s - Smart Controller Retrofit\n", DEVICE_MODEL);
    Serial.printf("  Firmware Version : %s\n", FIRMWARE_VERSION);
    Serial.println("  Milestones       : All 9 Phases Operational (Phases 1-9)");
    Serial.println("==================================================");
    Serial.printf("[DEVICE] Device ID      : %s\n", deviceId.c_str());
    Serial.printf("[DEVICE] MAC Address    : %s\n", macStr);
    Serial.printf("[CHIP]   Model          : %s (rev %d)\n", ESP.getChipModel(), ESP.getChipRevision());
    Serial.printf("[CHIP]   Cores / Clock  : %d cores @ %u MHz\n", chip_info.cores, getCpuFrequencyMhz());
    Serial.printf("[CHIP]   SDK Version    : %s\n", ESP.getSdkVersion());
    Serial.printf("[CHIP]   Reset Reason   : %s\n", getResetReasonString(rtc_get_reset_reason(0)));
    Serial.printf("[MEMORY] Flash Size     : %u MB\n", ESP.getFlashChipSize() / (1024 * 1024));
    Serial.printf("[MEMORY] Flash Speed    : %u MHz\n", ESP.getFlashChipSpeed() / 1000000);
    Serial.printf("[MEMORY] Free Heap      : %u bytes\n", ESP.getFreeHeap());
    Serial.printf("[MEMORY] Min Free Heap  : %u bytes\n", ESP.getMinFreeHeap());
    Serial.println("--------------------------------------------------");
    Serial.println("[SUBSYSTEM ARCHITECTURE - 9 MILESTONES]");
    Serial.printf("  Phase 1: Foundation     - ESP32 Boot, TWDT (%us), Unique ID\n", WDT_TIMEOUT_SECONDS);
    Serial.printf("  Phase 2: Climate Sensor - DHT22 on GPIO %d (Sample: %ums)\n", DHT_PIN, DHT_SAMPLE_INTERVAL_MS);
    Serial.printf("  Phase 3: IR Receiver    - 72-bit Azure Essence on GPIO %d\n", IR_RX_PIN);
    Serial.printf("  Phase 4: IR Transmitter - 38kHz Modulated on GPIO %d\n", IR_TX_PIN);
    Serial.println("  Phase 5: AC Model       - Azure Essence Split Inverter Controller");
    Serial.println("  Phase 6: State Machine  - Central ACState Manager & Serialization");
    Serial.printf("  Phase 7: Presence       - Obstacle/Proximity Sensor on GPIO %d\n", IR_OBSTACLE_PIN);
    Serial.println("  Phase 8: Energy Monitor - Isolated AC Real-Time Power/kWh/Cost Tracker");
    Serial.println("  Phase 9: Automation     - Deterministic Climate & Safety Engine");
    Serial.println("==================================================");
    Serial.println("[STREAM] Beginning telemetry stream...");
    Serial.println();
}

void TelemetryManager::printReadingJSON(const DHTReading& reading, const String& deviceId) {
    JsonDocument doc;

    if (reading.valid) {
        float roundedTemp = round(reading.temperature * 10.0f) / 10.0f;
        float roundedHum = round(reading.humidity * 10.0f) / 10.0f;
        doc["temperature"] = roundedTemp;
        doc["humidity"] = roundedHum;
        doc["temperature_c"] = roundedTemp;
        doc["humidity_percent"] = roundedHum;
        doc["heat_index_c"] = round(reading.heat_index_c * 10.0f) / 10.0f;
        doc["dew_point_c"] = round(reading.dew_point_c * 10.0f) / 10.0f;
        doc["comfort_status"] = reading.comfort_status;
        doc["mold_risk_score"] = round(reading.mold_risk_score);
        doc["mold_risk_level"] = reading.mold_risk_level;
        doc["thermal_rate_c_per_hr"] = round(reading.thermal_rate_c_per_hr * 10.0f) / 10.0f;
        doc["vpd_kpa"] = round(reading.vpd_kpa * 100.0f) / 100.0f;
    } else {
        doc["temperature"] = nullptr;
        doc["humidity"] = nullptr;
        doc["temperature_c"] = nullptr;
        doc["humidity_percent"] = nullptr;
        doc["heat_index_c"] = nullptr;
        doc["dew_point_c"] = nullptr;
    }

    doc["sensor_status"] = reading.status;

    if (reading.error_message != nullptr) {
        doc["error"] = reading.error_message;
    }

    doc["device_id"] = deviceId;
    doc["timestamp_ms"] = reading.timestamp_ms;
    doc["uptime_s"] = millis() / 1000;
    doc["free_heap"] = ESP.getFreeHeap();

    serializeJson(doc, Serial);
    Serial.println();
}

void TelemetryManager::printReadingJSON(const DHTReading& reading,
                                       const ac::control::ACState& acState,
                                       const ac::automation::AutomationEngine& autoEngine,
                                       bool isPresent,
                                       const String& deviceId) {
    JsonDocument doc;

    if (reading.valid) {
        float roundedTemp = round(reading.temperature * 10.0f) / 10.0f;
        float roundedHum = round(reading.humidity * 10.0f) / 10.0f;
        doc["temperature_c"] = roundedTemp;
        doc["humidity_percent"] = roundedHum;
        doc["heat_index_c"] = round(reading.heat_index_c * 10.0f) / 10.0f;
        doc["dew_point_c"] = round(reading.dew_point_c * 10.0f) / 10.0f;
        doc["comfort_status"] = reading.comfort_status;
        doc["mold_risk_score"] = round(reading.mold_risk_score);
        doc["mold_risk_level"] = reading.mold_risk_level;
        doc["thermal_rate_c_per_hr"] = round(reading.thermal_rate_c_per_hr * 10.0f) / 10.0f;
        doc["vpd_kpa"] = round(reading.vpd_kpa * 100.0f) / 100.0f;
    } else {
        doc["temperature_c"] = nullptr;
        doc["humidity_percent"] = nullptr;
        doc["heat_index_c"] = nullptr;
        doc["dew_point_c"] = nullptr;
    }

    doc["sensor_status"] = reading.status;
    doc["valid"] = reading.valid;
    if (reading.error_message != nullptr) {
        doc["error"] = reading.error_message;
    }

    // AC Digital Twin State
    doc["power"] = acState.power;
    doc["temperature"] = acState.temperature;
    doc["mode"] = acState.mode;
    doc["fan_speed"] = acState.fanSpeed;

    // Automation & Presence State
    doc["presence"] = isPresent;
    doc["presence_tier"] = autoEngine.getPresenceTierStr();
    doc["sleep_stage"] = autoEngine.getSleepStageStr();
    doc["sleep_enabled"] = autoEngine.isCircadianSleepEnabled();
    doc["auto_enabled"] = autoEngine.isEnabled();
    doc["thermal_breach"] = autoEngine.isThermalBreachActive();
    doc["thermal_breach_delta"] = autoEngine.getThermalBreachDelta();

    doc["device_id"] = deviceId;
    doc["timestamp_ms"] = reading.timestamp_ms > 0 ? reading.timestamp_ms : millis();
    doc["uptime_s"] = millis() / 1000;
    doc["free_heap"] = ESP.getFreeHeap();

    serializeJson(doc, Serial);
    Serial.println();
}

void TelemetryManager::printEnergyJSON(const ac::sensors::EnergyReading& energy, const String& deviceId) {
    JsonDocument doc;

    doc["type"] = "energy_telemetry";
    doc["device_id"] = deviceId;
    doc["voltage"] = round(energy.voltage * 10.0f) / 10.0f;
    doc["current"] = round(energy.current * 100.0f) / 100.0f;
    doc["power_watts"] = round(energy.power_watts * 10.0f) / 10.0f;
    doc["power_factor"] = round(energy.power_factor * 100.0f) / 100.0f;
    doc["energy_kwh_today"] = round(energy.energy_kwh_today * 10000.0f) / 10000.0f;
    doc["tariff_rate"] = round(energy.tariff_rate * 100.0f) / 100.0f;
    doc["estimated_cost_today"] = round(energy.estimated_cost_today * 100.0f) / 100.0f;
    doc["status"] = energy.status;
    doc["timestamp_ms"] = energy.timestamp_ms;
    doc["valid"] = energy.valid;

    serializeJson(doc, Serial);
    Serial.println();
}
