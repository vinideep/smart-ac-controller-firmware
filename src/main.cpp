#include <Arduino.h>
#include <esp_idf_version.h>
#include <esp_task_wdt.h>
#include "config/pins.h"
#include "config/config.h"
#include "sensors/dht_sensor.h"
#include "sensors/presence_sensor.h"
#include "sensors/energy_monitor.h"
#include "telemetry/telemetry.h"
#include "ir/ir_manager.h"
#include "ac/ac_controller.h"
#include "safety/safety_manager.h"
#include "automation/automation_engine.h"
#include "network/network_manager.h"
#include "network/cloud_client.h"
#include "api/local_api.h"

// Hardware driver and subsystem instances (All 9 Phases)
DHTDriver dhtDriver(DHT_PIN, DHT_TYPE);
ac::sensors::PresenceSensorDriver presenceSensor(IR_OBSTACLE_PIN);
ac::sensors::EnergyMonitor energyMonitor(8.0f, 230.0f); // Configurable tariff ₹8.0/kWh, nominal 230V
ac::ir::IRReceiverDriver irReceiver(IR_RX_PIN, ac::ir::kCaptureBufferSize, ac::ir::kTimeout);
ac::control::AzureEssenceController acController(IR_TX_PIN);
ac::safety::SafetyManager safetyManager(10000, 10000, 2000); // 10s min on/off for automation, 2s throttle
ac::automation::AutomationEngine automationEngine(acController, safetyManager, presenceSensor, dhtDriver);
// Device identity
String deviceId;

ac::network::NetworkManager networkManager;
ac::api::LocalAPIServer apiServer(acController, dhtDriver, presenceSensor, safetyManager, networkManager, deviceId);
ac::cloud::CloudClient cloudClient(BACKEND_URL, DEVICE_TOKEN);

// Serial command buffer
String serialInputBuffer = "";

void processCommand(const String& cmdStr);

void setup() {
    // 1. Initialize Serial communication
    Serial.begin(MONITOR_BAUD_RATE);
    delay(500); // Allow serial line to settle

    // 2. Configure Status LED
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);

    // 3. Derive unique Device ID from chip MAC address
    uint64_t mac = ESP.getEfuseMac();
    char idBuf[32];
    snprintf(idBuf, sizeof(idBuf), "esp32-%02x%02x%02x",
             (uint8_t)(mac >> 24),
             (uint8_t)(mac >> 32),
             (uint8_t)(mac >> 40));
    deviceId = String(idBuf);

    // 4. Initialize Hardware Task Watchdog Timer
#if ESP_IDF_VERSION_MAJOR >= 5
    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = WDT_TIMEOUT_SECONDS * 1000,
        .idle_core_mask = 0,
        .trigger_panic = true
    };
    esp_err_t err = esp_task_wdt_reconfigure(&twdt_config);
    if (err == ESP_ERR_INVALID_STATE) {
        esp_task_wdt_init(&twdt_config);
    }
    esp_task_wdt_add(NULL);
#else
    esp_task_wdt_init(WDT_TIMEOUT_SECONDS, true);
    esp_task_wdt_add(NULL);
#endif

    // 5. Initialize Subsystems (Phases 1 - 9)
    dhtDriver.begin();
    presenceSensor.begin();
    energyMonitor.begin();
    irReceiver.begin();
    acController.begin();
    automationEngine.begin();

    // 6. Initialize Network Manager (Wi-Fi STA / Fallback AP) & Local REST Server
    networkManager.begin(WIFI_SSID, WIFI_PASSWORD, WIFI_HOSTNAME);
    apiServer.begin(80);
    cloudClient.begin();

    // 7. Print boot banner and device information
    TelemetryManager::printDeviceInfo(deviceId);
    Serial.printf("[SYSTEM] IR Transmitter active on GPIO %d\n", IR_TX_PIN);
    Serial.printf("[SYSTEM] Presence/Obstacle sensor active on GPIO %d\n", IR_OBSTACLE_PIN);
    Serial.printf("[SYSTEM] Energy Monitor active (Nominal: %.0fV, Tariff: %.2f/kWh)\n",
                  energyMonitor.getNominalVoltage(), energyMonitor.getTariff());
    Serial.println("[SYSTEM] Ready. Commands: POWER_ON, POWER_OFF, SET_TEMP <16-31>, SET_MODE <COOL|DRY|FAN|AUTO>, SET_FAN <AUTO|MED|HIGH>, AUTO_ON, AUTO_OFF, GET_STATE, GET_PRESENCE, GET_ENERGY, SET_TARIFF <rate>, TEST_TX");

    // Visual boot indication (double blink)
    for (int i = 0; i < 2; i++) {
        digitalWrite(STATUS_LED_PIN, HIGH);
        delay(100);
        digitalWrite(STATUS_LED_PIN, LOW);
        delay(100);
    }
}

void processCommand(const String& rawCmd) {
    String cmd = rawCmd;
    cmd.trim();
    if (cmd.length() == 0) return;

    // Check for JSON commands
    if (cmd.startsWith("{")) {
        JsonDocument doc;
        DeserializationError err = deserializeJson(doc, cmd);
        if (!err) {
            const char* commandType = doc["cmd"] | "";
            if (strcmp(commandType, "power") == 0) {
                bool pwr = doc["value"] | false;
                digitalWrite(STATUS_LED_PIN, HIGH);
                acController.setPower(pwr, "json_cmd");
                safetyManager.recordPowerTransition(pwr, millis());
                digitalWrite(STATUS_LED_PIN, LOW);
                Serial.printf("[CMD_OK] AC Power set to %s\n", pwr ? "ON" : "OFF");
            } else if (strcmp(commandType, "temp") == 0) {
                uint8_t t = doc["value"] | 25;
                acController.setTemperature(t, "serial_json");
                safetyManager.recordCommandSent(millis());
            } else if (strcmp(commandType, "mode") == 0) {
                const char* m = doc["value"] | "cool";
                digitalWrite(STATUS_LED_PIN, HIGH);
                acController.setMode(String(m), "serial_json");
                safetyManager.recordCommandSent(millis());
                digitalWrite(STATUS_LED_PIN, LOW);
                Serial.printf("[CMD_OK] AC Mode set to %s\n", m);
            } else if (strcmp(commandType, "fan") == 0) {
                const char* spd = doc["value"] | "auto";
                acController.setFanSpeed(String(spd), "serial_json");
                safetyManager.recordCommandSent(millis());
            } else if (strcmp(commandType, "auto") == 0) {
                bool en = doc["value"] | false;
                automationEngine.setEnabled(en);
            } else if (strcmp(commandType, "sleep") == 0) {
                bool en = doc["value"] | false;
                uint8_t pdown = doc["pulldown"] | 23;
                float ramp = doc["ramp"] | 0.5f;
                float maxT = doc["max_temp"] | 25.5f;
                automationEngine.setCircadianSleep(en, pdown, ramp, maxT);
            } else if (strcmp(commandType, "eco_drift") == 0) {
                bool en = doc["value"] | false;
                automationEngine.setEcoDriftEnabled(en);
            } else if (strcmp(commandType, "psychro") == 0) {
                bool en = doc["value"] | false;
                automationEngine.setPsychrometricEnabled(en);
            } else if (strcmp(commandType, "clear_breach") == 0) {
                automationEngine.clearThermalBreach();
                Serial.println("[CMD_OK] Thermal breach alert cleared");
            } else if (strcmp(commandType, "auto_config") == 0) {
                if (!doc["enabled"].isNull()) automationEngine.setEnabled(doc["enabled"].as<bool>());
                if (!doc["target_temp"].isNull()) automationEngine.setTargetTemperature(doc["target_temp"].as<float>());
                if (!doc["hysteresis"].isNull()) automationEngine.setHysteresis(doc["hysteresis"].as<float>());
                if (!doc["empty_timeout_s"].isNull()) automationEngine.setEmptyTimeoutSeconds(doc["empty_timeout_s"].as<uint32_t>());
                if (!doc["eco_drift_timeout_s"].isNull()) automationEngine.setEcoDriftTimeoutSeconds(doc["eco_drift_timeout_s"].as<uint32_t>());
                if (!doc["eco_drift_enabled"].isNull()) automationEngine.setEcoDriftEnabled(doc["eco_drift_enabled"].as<bool>());
                if (!doc["psychrometric_enabled"].isNull()) automationEngine.setPsychrometricEnabled(doc["psychrometric_enabled"].as<bool>());
                if (!doc["dry_mode_humidity_threshold"].isNull()) automationEngine.setDryModeHumidityThreshold(doc["dry_mode_humidity_threshold"].as<float>());
                if (!doc["thermal_breach_protection"].isNull()) automationEngine.setThermalBreachProtection(doc["thermal_breach_protection"].as<bool>());
                if (!doc["presence_detection_enabled"].isNull()) automationEngine.setPresenceDetectionEnabled(doc["presence_detection_enabled"].as<bool>());
                if (!doc["sleep_enabled"].isNull()) {
                    automationEngine.setCircadianSleep(doc["sleep_enabled"].as<bool>(), doc["pulldown_temp"] | 23, doc["ramp_rate"] | 0.5f, doc["max_temp"] | 25.5f);
                }
                Serial.println("[CMD_OK] Automation engine configuration updated");
            } else if (strcmp(commandType, "state") == 0) {
                Serial.println(acController.getState().toJSONString());
            } else if (strcmp(commandType, "energy") == 0) {
                Serial.println(energyMonitor.toJSONString());
            } else if (strcmp(commandType, "tariff") == 0) {
                float rate = doc["value"] | 0.0f;
                if (energyMonitor.setTariff(rate)) {
                    Serial.printf("[CMD_OK] Energy tariff set to %.2f per kWh\n", rate);
                } else {
                    Serial.println("[CMD_ERR] Invalid tariff rate");
                }
            } else if (strcmp(commandType, "send_raw") == 0 || strcmp(commandType, "raw") == 0) {
                JsonArray rawArr = doc["raw"].as<JsonArray>();
                uint16_t freq = doc["freq"] | 38;
                uint16_t len = rawArr.size();
                if (len > 0 && len <= 350) {
                    uint16_t rawBuf[350];
                    for (uint16_t i = 0; i < len; i++) {
                        rawBuf[i] = rawArr[i];
                    }
                    digitalWrite(STATUS_LED_PIN, HIGH);
                    acController.getTransmitter().sendRaw(rawBuf, len, freq);
                    safetyManager.recordCommandSent(millis());
                    digitalWrite(STATUS_LED_PIN, LOW);
                    Serial.printf("[CMD_OK] Transmitted %u raw transitions at %ukHz\n", len, freq);
                }
            }
            return;
        }
    }

    // String commands
    String upper = cmd;
    upper.toUpperCase();

    if (upper == "POWER_ON" || upper == "ON") {
        digitalWrite(STATUS_LED_PIN, HIGH);
        acController.setPower(true, "manual_cmd");
        safetyManager.recordPowerTransition(true, millis());
        digitalWrite(STATUS_LED_PIN, LOW);
        Serial.println("[CMD_OK] AC Power ON sent");
    } else if (upper == "POWER_OFF" || upper == "OFF") {
        digitalWrite(STATUS_LED_PIN, HIGH);
        acController.setPower(false, "manual_cmd");
        safetyManager.recordPowerTransition(false, millis());
        digitalWrite(STATUS_LED_PIN, LOW);
        Serial.println("[CMD_OK] AC Power OFF sent");
    } else if (upper.startsWith("SET_TEMP")) {
        int spaceIdx = upper.indexOf(' ');
        if (spaceIdx > 0) {
            int tempVal = upper.substring(spaceIdx + 1).toInt();
            if (tempVal >= ac::control::AzureEssenceController::kMinTemperature &&
                tempVal <= ac::control::AzureEssenceController::kMaxTemperature) {
                digitalWrite(STATUS_LED_PIN, HIGH);
                acController.setTemperature((uint8_t)tempVal, "serial");
                safetyManager.recordCommandSent(millis());
                digitalWrite(STATUS_LED_PIN, LOW);
                Serial.printf("[CMD_OK] AC Temperature set to %d C\n", tempVal);
            } else {
                Serial.println("[CMD_ERR] Temperature must be between 16 and 31 C");
            }
        } else {
            Serial.println("[CMD_ERR] Usage: SET_TEMP <16-31>");
        }
    } else if (upper.startsWith("SET_MODE")) {
        int spaceIdx = upper.indexOf(' ');
        if (spaceIdx > 0) {
            String m = upper.substring(spaceIdx + 1);
            m.toLowerCase();
            if (m == "cool" || m == "dry" || m == "fan" || m == "auto") {
                digitalWrite(STATUS_LED_PIN, HIGH);
                acController.setMode(m, "serial");
                safetyManager.recordCommandSent(millis());
                digitalWrite(STATUS_LED_PIN, LOW);
                Serial.printf("[CMD_OK] AC Mode set to %s\n", m.c_str());
            } else {
                Serial.println("[CMD_ERR] Mode must be COOL, DRY, FAN, or AUTO");
            }
        } else {
            Serial.println("[CMD_ERR] Usage: SET_MODE <COOL|DRY|FAN|AUTO>");
        }
    } else if (upper.startsWith("SET_FAN")) {
        int spaceIdx = upper.indexOf(' ');
        if (spaceIdx > 0) {
            String spd = upper.substring(spaceIdx + 1);
            spd.toLowerCase();
            digitalWrite(STATUS_LED_PIN, HIGH);
            acController.setFanSpeed(spd, "serial");
            safetyManager.recordCommandSent(millis());
            digitalWrite(STATUS_LED_PIN, LOW);
            Serial.printf("[CMD_OK] AC Fan set to %s\n", spd.c_str());
        } else {
            Serial.println("[CMD_ERR] Usage: SET_FAN <AUTO|MED|HIGH>");
        }
    } else if (upper == "AUTO_ON") {
        automationEngine.setEnabled(true);
        Serial.println("[CMD_OK] Local climate automation ENABLED");
    } else if (upper == "AUTO_OFF") {
        automationEngine.setEnabled(false);
        Serial.println("[CMD_OK] Local climate automation DISABLED");
    } else if (upper.startsWith("SET_TARGET")) {
        int spaceIdx = upper.indexOf(' ');
        if (spaceIdx > 0) {
            float target = upper.substring(spaceIdx + 1).toFloat();
            automationEngine.setTargetTemperature(target);
            Serial.printf("[CMD_OK] Automation target temperature set to %.1f C\n", target);
        } else {
            Serial.println("[CMD_ERR] Usage: SET_TARGET <temp>");
        }
    } else if (upper.startsWith("SLEEP_ON")) {
        int spaceIdx = upper.indexOf(' ');
        uint8_t pdown = 23;
        if (spaceIdx > 0) {
            pdown = upper.substring(spaceIdx + 1).toInt();
            if (pdown < 18 || pdown > 28) pdown = 23;
        }
        automationEngine.setCircadianSleep(true, pdown, 0.5f, 25.5f);
        Serial.printf("[CMD_OK] Circadian metabolic sleep schedule ACTIVE (Pulldown: %d C)\n", pdown);
    } else if (upper == "SLEEP_OFF") {
        automationEngine.setCircadianSleep(false);
        Serial.println("[CMD_OK] Circadian sleep schedule DISABLED");
    } else if (upper == "ECO_ON") {
        automationEngine.setEcoDriftEnabled(true);
        Serial.println("[CMD_OK] Presence Eco-Drift (+1C drift) ENABLED");
    } else if (upper == "ECO_OFF") {
        automationEngine.setEcoDriftEnabled(false);
        Serial.println("[CMD_OK] Presence Eco-Drift DISABLED");
    } else if (upper == "PSYCHRO_ON") {
        automationEngine.setPsychrometricEnabled(true);
        Serial.println("[CMD_OK] Psychrometric comfort engine & autonomous mode arbitration ENABLED");
    } else if (upper == "PSYCHRO_OFF") {
        automationEngine.setPsychrometricEnabled(false);
        Serial.println("[CMD_OK] Psychrometric comfort engine DISABLED");
    } else if (upper == "CLEAR_BREACH") {
        automationEngine.clearThermalBreach();
        Serial.println("[CMD_OK] Active thermal breach cleared");
    } else if (upper == "BREACH_ON") {
        automationEngine.setThermalBreachProtection(true);
        Serial.println("[CMD_OK] Thermal breach protection ENABLED");
    } else if (upper == "BREACH_OFF") {
        automationEngine.setThermalBreachProtection(false);
        Serial.println("[CMD_OK] Thermal breach protection DISABLED");
    } else if (upper == "GET_STATE" || upper == "STATE") {
        Serial.println(acController.getState().toJSONString());
    } else if (upper == "GET_PRESENCE") {
        JsonDocument doc;
        presenceSensor.toJSON(doc);
        String out;
        serializeJson(doc, out);
        Serial.println(out);
    } else if (upper == "GET_AUTO") {
        JsonDocument doc;
        automationEngine.toJSON(doc);
        String out;
        serializeJson(doc, out);
        Serial.println(out);
    } else if (upper == "GET_ENERGY" || upper == "ENERGY") {
        Serial.println(energyMonitor.toJSONString());
    } else if (upper == "RESET_ENERGY" || upper == "RESET_DAILY") {
        energyMonitor.resetDaily();
        Serial.println("[CMD_OK] Daily energy and cost counters reset to zero");
    } else if (upper.startsWith("SET_TARIFF")) {
        int spaceIdx = upper.indexOf(' ');
        if (spaceIdx > 0) {
            float rate = upper.substring(spaceIdx + 1).toFloat();
            if (energyMonitor.setTariff(rate)) {
                Serial.printf("[CMD_OK] Energy tariff set to %.2f per kWh\n", rate);
            } else {
                Serial.println("[CMD_ERR] Tariff rate must be greater than 0");
            }
        } else {
            Serial.println("[CMD_ERR] Usage: SET_TARIFF <rate>");
        }
    } else if (upper == "TEST_TX") {
        digitalWrite(STATUS_LED_PIN, HIGH);
        acController.getTransmitter().testPowerOn();
        safetyManager.recordPowerTransition(true, millis());
        digitalWrite(STATUS_LED_PIN, LOW);
        Serial.println("[CMD_OK] Transmitted Test Power ON frame (Cool, 25C, Auto Fan) with repeat");
    } else {
        Serial.printf("[CMD_UNKNOWN] Unrecognized command: %s\n", cmd.c_str());
    }
}


void loop() {
    // 1. Reset Watchdog Timer
    esp_task_wdt_reset();

    // 2. Poll IR Learning Receiver (non-blocking) & Mirror AC State
    if (irReceiver.update(deviceId)) {
        if (irReceiver.hasLastCapture()) {
            const ac::ir::IRCaptureInfo& cap = irReceiver.getLastCapture();
            if (cap.hasAcState) {
                acController.applyExternalState(cap.acState);
                safetyManager.recordPowerTransition(cap.acState.power, millis());
                safetyManager.recordCommandSent(millis());
                JsonDocument syncDoc;
                syncDoc["device_id"] = deviceId;
                syncDoc["power"] = cap.acState.power;
                syncDoc["temperature"] = cap.acState.temperature;
                syncDoc["mode"] = cap.acState.mode;
                syncDoc["fan_speed"] = cap.acState.fanSpeed;
                syncDoc["source"] = "ir_remote";
                syncDoc["timestamp_ms"] = millis();
                serializeJson(syncDoc, Serial);
                Serial.println();
                if (cloudClient.isEnabled()) {
                    cloudClient.update(syncDoc);
                }
            }
            JsonDocument irDoc;
            irDoc["device_id"] = deviceId;
            irDoc["type"] = "ir_capture";
            irDoc["protocol"] = cap.protocol;
            irDoc["bits"] = cap.bits;
            irDoc["hex"] = cap.hexCode;
            irDoc["is_ac"] = cap.isAc;
            if (cap.hasAcState) {
                irDoc["ac_power"] = cap.acState.power;
                irDoc["ac_temp"] = cap.acState.temperature;
                irDoc["ac_mode"] = cap.acState.mode;
                irDoc["ac_fan"] = cap.acState.fanSpeed;
            }
            if (cap.rawLength > 0) {
                JsonArray rawArr = irDoc["raw"].to<JsonArray>();
                for (uint16_t r = 0; r < cap.rawLength; r++) {
                    rawArr.add(cap.rawData[r]);
                }
            }
            irDoc["timestamp_ms"] = cap.timestampMs;
            serializeJson(irDoc, Serial);
            Serial.println();
            if (cloudClient.isEnabled()) {
                cloudClient.update(irDoc);
            }
            irReceiver.clearLastCapture();
        }
    }

    // 3. Update Presence Sensor Driver (only if hardware physically installed)
#if HAS_PRESENCE_SENSOR
    if (presenceSensor.update()) {
        JsonDocument doc;
        presenceSensor.toJSON(doc);
        doc["type"] = "presence_event";
        doc["device_id"] = deviceId;
        doc["timestamp_ms"] = millis();
        String out;
        serializeJson(doc, out);
        Serial.println(out);
    }
#endif

    // 4. Update Local Automation Engine
    automationEngine.update();

    // 5. Update Energy Monitoring Integration (accumulate continuously, refresh AC load model at 1Hz)
    static uint32_t lastEnergyModelUpdate = 0;
    uint32_t currentNow = millis();
    energyMonitor.update(currentNow);
    if (currentNow - lastEnergyModelUpdate >= 1000) {
        lastEnergyModelUpdate = currentNow;
        float currentAmbientTemp = dhtDriver.getLatestReading().valid ? dhtDriver.getLatestReading().temperature_c : 25.0f;
        energyMonitor.updateFromAcState(
            acController.getState().power,
            acController.getState().mode,
            acController.getState().fanSpeed,
            acController.getState().temperature,
            currentAmbientTemp,
            currentNow
        );
    }

    // 6. Update Network Manager & Local REST API Server
    networkManager.update();
    apiServer.update();

    // 7. Process Serial Input Commands
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (serialInputBuffer.length() > 0) {
                processCommand(serialInputBuffer);
                serialInputBuffer = "";
            }
        } else if (serialInputBuffer.length() < 256) {
            serialInputBuffer += c;
        }
    }

    // 8. Update Cloud Client (push telemetry & poll commands)
    if (cloudClient.isEnabled()) {
        JsonDocument cloudDoc;
        const DHTReading& dhtReading = dhtDriver.getLatestReading();
        cloudDoc["device_id"] = deviceId;
        cloudDoc["temperature_c"] = dhtReading.temperature_c;
        cloudDoc["humidity_percent"] = dhtReading.humidity_percent;
        cloudDoc["heat_index_c"] = dhtReading.heat_index_c;
        cloudDoc["dew_point_c"] = dhtReading.dew_point_c;
        cloudDoc["comfort_status"] = dhtReading.comfort_status;
        cloudDoc["mold_risk_score"] = dhtReading.mold_risk_score;
        cloudDoc["mold_risk_level"] = dhtReading.mold_risk_level;
        cloudDoc["thermal_rate_c_per_hr"] = dhtReading.thermal_rate_c_per_hr;
        cloudDoc["vpd_kpa"] = dhtReading.vpd_kpa;
        cloudDoc["valid"] = dhtReading.valid;
        cloudDoc["sensor_status"] = dhtReading.valid ? "ok" : "stale";
        cloudDoc["power"] = acController.getState().power;
        cloudDoc["temperature"] = acController.getState().temperature;
        cloudDoc["mode"] = acController.getState().mode;
        cloudDoc["fan_speed"] = acController.getState().fanSpeed;
        cloudDoc["presence"] = HAS_PRESENCE_SENSOR ? presenceSensor.isPresent() : false;
        cloudDoc["presence_installed"] = HAS_PRESENCE_SENSOR;
        cloudDoc["presence_tier"] = automationEngine.getPresenceTierStr();
        cloudDoc["sleep_stage"] = automationEngine.getSleepStageStr();
        cloudDoc["sleep_enabled"] = automationEngine.isCircadianSleepEnabled();
        cloudDoc["thermal_breach"] = automationEngine.isThermalBreachActive();
        cloudDoc["thermal_breach_delta"] = automationEngine.getThermalBreachDelta();
        cloudDoc["eco_drift_enabled"] = automationEngine.isEcoDriftEnabled();
        cloudDoc["psychrometric_enabled"] = automationEngine.isPsychrometricEnabled();
        cloudDoc["auto_enabled"] = automationEngine.isEnabled();
        cloudDoc["presence_detection_enabled"] = automationEngine.isPresenceDetectionEnabled();
        cloudDoc["ir_tx_installed"] = HAS_IR_TRANSMITTER;
        cloudDoc["ir_rx_installed"] = true;
        cloudDoc["dht22_installed"] = true;
        cloudDoc["timestamp_ms"] = millis();

        cloudClient.update(cloudDoc);

        bool hadCmd = false;
        while (cloudClient.hasCommand()) {
            String cloudCmd = cloudClient.dequeueCommand();
            Serial.printf("[CLOUD] Executing: %s\n", cloudCmd.c_str());
            processCommand(cloudCmd);
            hadCmd = true;
        }
        if (hadCmd) {
            cloudDoc["power"] = acController.getState().power;
            cloudDoc["temperature"] = acController.getState().temperature;
            cloudDoc["mode"] = acController.getState().mode;
            cloudDoc["fan_speed"] = acController.getState().fanSpeed;
            cloudClient.update(cloudDoc);
        }
    }

    // 7. Update DHT22 driver (samples every DHT_SAMPLE_INTERVAL_MS non-blockingly)
    if (dhtDriver.update()) {
        const DHTReading& reading = dhtDriver.getLatestReading();

        // Visual heartbeat blink on sensor read
        digitalWrite(STATUS_LED_PIN, HIGH);
        delay(30);
        digitalWrite(STATUS_LED_PIN, LOW);

        // Emit clean JSON climate & energy telemetry
        TelemetryManager::printReadingJSON(reading, acController.getState(), automationEngine, HAS_PRESENCE_SENSOR ? presenceSensor.isPresent() : false, deviceId);
        TelemetryManager::printEnergyJSON(energyMonitor.getReading(), deviceId);
    }

    // Yield to FreeRTOS scheduler
    delay(5);
}
