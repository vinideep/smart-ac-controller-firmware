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
ac::safety::SafetyManager safetyManager(180000, 180000, 5000); // 3m min on/off, 5s throttle
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
    Serial.println("[SYSTEM] Ready. Commands: POWER_ON, POWER_OFF, SET_TEMP <16-31>, SET_FAN <AUTO|MED|HIGH>, AUTO_ON, AUTO_OFF, GET_STATE, GET_PRESENCE, GET_ENERGY, SET_TARIFF <rate>, TEST_TX");

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
                const char* reason = nullptr;
                if (pwr ? safetyManager.canTurnOn(millis(), reason) : safetyManager.canTurnOff(millis(), reason)) {
                    acController.setPower(pwr, "serial_json");
                    safetyManager.recordPowerTransition(pwr, millis());
                } else {
                    Serial.printf("[SAFETY_BLOCKED] %s\n", reason ? reason : "rejected");
                }
            } else if (strcmp(commandType, "temp") == 0) {
                uint8_t t = doc["value"] | 25;
                acController.setTemperature(t, "serial_json");
                safetyManager.recordCommandSent(millis());
            } else if (strcmp(commandType, "fan") == 0) {
                const char* spd = doc["value"] | "auto";
                acController.setFanSpeed(String(spd), "serial_json");
                safetyManager.recordCommandSent(millis());
            } else if (strcmp(commandType, "auto") == 0) {
                bool en = doc["value"] | false;
                automationEngine.setEnabled(en);
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
            }
            return;
        }
    }

    // String commands
    String upper = cmd;
    upper.toUpperCase();

    if (upper == "POWER_ON" || upper == "ON") {
        const char* reason = nullptr;
        if (safetyManager.canTurnOn(millis(), reason)) {
            digitalWrite(STATUS_LED_PIN, HIGH);
            acController.setPower(true, "serial");
            safetyManager.recordPowerTransition(true, millis());
            digitalWrite(STATUS_LED_PIN, LOW);
            Serial.println("[CMD_OK] AC Power ON sent");
        } else {
            Serial.printf("[SAFETY_BLOCKED] %s\n", reason ? reason : "rejected");
        }
    } else if (upper == "POWER_OFF" || upper == "OFF") {
        const char* reason = nullptr;
        if (safetyManager.canTurnOff(millis(), reason)) {
            digitalWrite(STATUS_LED_PIN, HIGH);
            acController.setPower(false, "serial");
            safetyManager.recordPowerTransition(false, millis());
            digitalWrite(STATUS_LED_PIN, LOW);
            Serial.println("[CMD_OK] AC Power OFF sent");
        } else {
            Serial.printf("[SAFETY_BLOCKED] %s\n", reason ? reason : "rejected");
        }
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

    // 2. Poll IR Learning Receiver (non-blocking)
    irReceiver.update(deviceId);

    // 3. Update Presence Sensor Driver
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
        cloudDoc["valid"] = dhtReading.valid;
        cloudDoc["sensor_status"] = dhtReading.valid ? "ok" : "stale";
        cloudDoc["power"] = acController.getState().power;
        cloudDoc["temperature"] = acController.getState().temperature;
        cloudDoc["mode"] = acController.getState().mode;
        cloudDoc["fan_speed"] = acController.getState().fanSpeed;
        cloudDoc["presence"] = presenceSensor.isPresent();
        cloudDoc["timestamp_ms"] = millis();

        cloudClient.update(cloudDoc);

        while (cloudClient.hasCommand()) {
            String cloudCmd = cloudClient.dequeueCommand();
            Serial.printf("[CLOUD] Executing: %s\n", cloudCmd.c_str());
            processCommand(cloudCmd);
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
        TelemetryManager::printReadingJSON(reading, deviceId);
        TelemetryManager::printEnergyJSON(energyMonitor.getReading(), deviceId);
    }

    // Yield to FreeRTOS scheduler
    delay(5);
}
