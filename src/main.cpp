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
#include "ac/closed_loop_feedback.h"
#include "safety/safety_manager.h"
#include "automation/automation_engine.h"
#include "network/network_manager.h"
#include "network/cloud_client.h"
#include "api/local_api.h"
#include "ota/ota_manager.h"

// Hardware driver and subsystem instances (All 9 Phases)
DHTDriver dhtDriver(DHT_PIN, DHT_TYPE);
ac::sensors::PresenceSensorDriver presenceSensor(PRESENCE_PIN, PRESENCE_ACTIVE_LOW, RADAR_RX_PIN, RADAR_TX_PIN, RADAR_BAUD_RATE);
ac::sensors::EnergyMonitor energyMonitor(8.0f, 230.0f); // Configurable tariff ₹8.0/kWh, nominal 230V
ac::ir::IRReceiverDriver irReceiver(IR_RX_PIN, ac::ir::kCaptureBufferSize, ac::ir::kTimeout);
ac::control::AzureEssenceController acController(IR_TX_PIN);
ac::control::ClosedLoopFeedback closedLoopFeedback(acController, energyMonitor);
ac::safety::SafetyManager safetyManager(180000, 300000, 2000); // 3m min ON, 5m min OFF compressor protection, 2s throttle
ac::automation::AutomationEngine automationEngine(acController, safetyManager, presenceSensor, dhtDriver);
// Device identity
String deviceId;

ac::network::NetworkManager networkManager;
ac::api::LocalAPIServer apiServer(acController, dhtDriver, presenceSensor, safetyManager, networkManager, automationEngine, deviceId);
ac::cloud::CloudClient cloudClient(BACKEND_URL, DEVICE_TOKEN);

// Serial command buffer and TX blanking timestamp
String serialInputBuffer = "";
uint32_t lastTxBlankingTime = 0;

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
    acController.getTransmitter().setReceiver(&irReceiver);
    irReceiver.setTransmitter(&acController.getTransmitter());
    automationEngine.begin();
    closedLoopFeedback.begin();

    // 6. Initialize Network Manager (Wi-Fi STA / Fallback AP) & Local REST Server
    networkManager.begin(WIFI_SSID, WIFI_PASSWORD, WIFI_HOSTNAME);
    apiServer.begin(80);
    cloudClient.setTimeSyncCallback([](uint32_t epochSec, int16_t tzOffsetMin) {
        automationEngine.syncTime(epochSec, tzOffsetMin);
    });
    cloudClient.begin();

    // 7. Print boot banner and device information
    TelemetryManager::printDeviceInfo(deviceId);
    Serial.printf("[SYSTEM] IR Transmitter active on GPIO %d (Status: %s)\n", IR_TX_PIN, HAS_IR_TRANSMITTER ? "ENABLED" : "DISABLED");
    Serial.printf("[SYSTEM] Radar Presence sensor active on GPIO %d (Active %s, Status: %s)\n",
                  PRESENCE_PIN, PRESENCE_ACTIVE_LOW ? "LOW" : "HIGH", HAS_PRESENCE_SENSOR ? "ENABLED" : "DISABLED");
    Serial.printf("[SYSTEM] Energy Monitor active (Nominal: %.0fV, Tariff: %.2f/kWh)\n",
                  energyMonitor.getNominalVoltage(), energyMonitor.getTariff());
    Serial.println("[SYSTEM] Ready. Commands: POWER_ON, POWER_OFF, SET_TEMP <16-31>, SET_MODE <COOL|DRY|FAN|AUTO>, SET_FAN <AUTO|MED|HIGH>, SET_AC_STATE <1|0> <temp> <mode> <fan>, AUTO_ON, AUTO_OFF, GET_STATE, GET_PRESENCE, GET_ENERGY, SET_TARIFF <rate>, SET_PRESENCE_POLARITY <HIGH|LOW>, TEST_TX, SET_WIFI <ssid> [pass], WIFI_SCAN, WIFI_RESET");

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
            if (strcmp(commandType, "set_ac_state") == 0 || strcmp(commandType, "ac_state") == 0) {
                bool pwr = doc["power"] | false;
                uint8_t temp = doc["temp"] | (doc["temperature"] | 25);
                const char* m = doc["mode"] | "cool";
                const char* f = doc["fan"] | (doc["fan_speed"] | "auto");
                digitalWrite(STATUS_LED_PIN, HIGH);
                if (acController.setState(pwr, temp, String(m), String(f), "json_cmd")) {
                    lastTxBlankingTime = millis();
                    safetyManager.recordPowerTransition(pwr, millis());
                    safetyManager.recordCommandSent(millis());
                    closedLoopFeedback.notifyCommandSent(acController.getState(), millis());
                    automationEngine.setUserManualPowerOff(!pwr);
                    digitalWrite(STATUS_LED_PIN, LOW);
                    Serial.printf("[CMD_OK] AC state set: Power=%d Temp=%d Mode=%s Fan=%s\n",
                                  pwr ? 1 : 0, temp, m, f);
                } else {
                    digitalWrite(STATUS_LED_PIN, LOW);
                    Serial.println("[CMD_ERR] Invalid state parameters");
                }
            } else if (strcmp(commandType, "power") == 0) {
                bool pwr = doc["value"] | false;
                digitalWrite(STATUS_LED_PIN, HIGH);
                lastTxBlankingTime = millis();
                acController.setPower(pwr, "json_cmd");
                safetyManager.recordPowerTransition(pwr, millis());
                closedLoopFeedback.notifyCommandSent(acController.getState(), millis());
                automationEngine.setUserManualPowerOff(!pwr);
                digitalWrite(STATUS_LED_PIN, LOW);
                Serial.printf("[CMD_OK] AC Power set to %s\n", pwr ? "ON" : "OFF");
            } else if (strcmp(commandType, "temp") == 0) {
                uint8_t t = doc["value"] | 25;
                lastTxBlankingTime = millis();
                acController.setTemperature(t, "serial_json");
                safetyManager.recordCommandSent(millis());
                closedLoopFeedback.notifyCommandSent(acController.getState(), millis());
            } else if (strcmp(commandType, "mode") == 0) {
                const char* m = doc["value"] | "cool";
                digitalWrite(STATUS_LED_PIN, HIGH);
                lastTxBlankingTime = millis();
                acController.setMode(String(m), "serial_json");
                safetyManager.recordCommandSent(millis());
                closedLoopFeedback.notifyCommandSent(acController.getState(), millis());
                digitalWrite(STATUS_LED_PIN, LOW);
                Serial.printf("[CMD_OK] AC Mode set to %s\n", m);
            } else if (strcmp(commandType, "fan") == 0) {
                const char* spd = doc["value"] | "auto";
                lastTxBlankingTime = millis();
                acController.setFanSpeed(String(spd), "serial_json");
                safetyManager.recordCommandSent(millis());
                closedLoopFeedback.notifyCommandSent(acController.getState(), millis());
            } else if (strcmp(commandType, "auto") == 0) {
                bool en = doc["value"] | false;
                automationEngine.setEnabled(en);
            } else if (strcmp(commandType, "sleep") == 0) {
                bool en = doc["value"] | false;
                uint8_t pdown = doc["pulldown"] | 23;
                float ramp = doc["ramp"] | 0.5f;
                float maxT = doc["max_temp"] | 25.5f;
                automationEngine.setCircadianSleep(en, pdown, ramp, maxT);
            } else if (strcmp(commandType, "night_cycle") == 0) {
                bool en = !doc["enabled"].isNull() ? doc["enabled"].as<bool>() : (!doc["value"].isNull() ? doc["value"].as<bool>() : false);
                float tgt = doc["target_temp"] | doc["target"] | 27.0f;
                int8_t h = doc["hour"] | -1;
                int8_t m = doc["min"] | -1;
                if (!doc["epoch"].isNull()) {
                    uint32_t ep = doc["epoch"].as<uint32_t>();
                    int16_t tzOff = doc["tz_offset_min"] | 330;
                    automationEngine.syncTime(ep, tzOff);
                }
                automationEngine.setNightCycle(en, tgt, h, m);
                Serial.printf("[CMD_OK] Night sleep cycle %s (Target: %.1fC, Location time: %02u:%02u)\n",
                              en ? "ENABLED" : "DISABLED", tgt, automationEngine.getLocalHour(), automationEngine.getLocalMinute());
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
                if (!doc["night_cycle_enabled"].isNull()) {
                    float tgt = doc["night_cycle_target"] | doc["target_temp"] | 27.0f;
                    automationEngine.setNightCycle(doc["night_cycle_enabled"].as<bool>(), tgt);
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
                    lastTxBlankingTime = millis();
                    acController.getTransmitter().sendRaw(rawBuf, len, freq);
                    safetyManager.recordCommandSent(millis());
                    digitalWrite(STATUS_LED_PIN, LOW);
                    Serial.printf("[CMD_OK] Transmitted %u raw transitions at %ukHz\n", len, freq);
                }
            } else if (strcmp(commandType, "presence_polarity") == 0) {
                bool activeLow = doc["active_low"] | false;
                presenceSensor.setActiveLow(activeLow);
                Serial.printf("[CMD_OK] Presence sensor polarity set to Active-%s\n", activeLow ? "LOW" : "HIGH");
            } else if (strcmp(commandType, "radar_gates") == 0) {
                float minM = doc["min_m"] | 0.2f;
                float maxM = doc["max_m"] | 4.5f;
                presenceSensor.setDistanceGates(minM, maxM);
                Serial.printf("[CMD_OK] Radar gates set to %.2f - %.2f m\n", minM, maxM);
            } else if (strcmp(commandType, "radar_energy") == 0) {
                uint8_t moveE = doc["min_moving"] | 15;
                uint8_t statE = doc["min_stationary"] | 15;
                presenceSensor.setEnergyThresholds(moveE, statE);
                Serial.printf("[CMD_OK] Radar energy thresholds set to Move=%u%% Stat=%u%%\n", moveE, statE);
            } else if (strcmp(commandType, "absence_timeout") == 0) {
                uint32_t to = doc["value"] | 600;
                presenceSensor.setAbsenceTimeoutSeconds(to);
                Serial.printf("[CMD_OK] Radar absence debounce timeout set to %u s\n", to);
            } else if (strcmp(commandType, "closed_loop") == 0) {
                if (!doc["enabled"].isNull()) closedLoopFeedback.setEnabled(doc["enabled"].as<bool>());
                if (!doc["verify_timeout_s"].isNull()) closedLoopFeedback.setVerificationTimeoutSeconds(doc["verify_timeout_s"].as<uint32_t>());
                if (!doc["max_retries"].isNull()) closedLoopFeedback.setMaxRetries(doc["max_retries"].as<uint8_t>());
                Serial.println("[CMD_OK] Closed-loop configuration updated");
            } else if (strcmp(commandType, "comfort_opt") == 0) {
                bool en = doc["value"] | false;
                automationEngine.setComfortIndexOptimization(en);
                Serial.printf("[CMD_OK] Comfort index optimization set to %s\n", en ? "ENABLED" : "DISABLED");
            } else if (strcmp(commandType, "get_closed_loop") == 0) {
                JsonDocument clDoc;
                closedLoopFeedback.toJSON(clDoc);
                String clOut;
                serializeJson(clDoc, clOut);
                Serial.println(clOut);
            } else if (strcmp(commandType, "wifi_configure") == 0 || strcmp(commandType, "configure_wifi") == 0) {
                const char* s = doc["ssid"] | "";
                const char* p = doc["password"] | "";
                if (strlen(s) > 0) {
                    networkManager.configureWifi(String(s), String(p));
                    Serial.printf("[CMD_OK] Configured Wi-Fi for SSID: %s\n", s);
                } else {
                    Serial.println("[CMD_ERR] SSID cannot be empty");
                }
            } else if (strcmp(commandType, "wifi_scan") == 0) {
                std::vector<ac::network::ScannedNetwork> nets = networkManager.scanNetworks();
                JsonDocument scanDoc;
                scanDoc["type"] = "wifi_scan_results";
                scanDoc["device_id"] = deviceId;
                JsonArray arr = scanDoc["networks"].to<JsonArray>();
                for (const auto& n : nets) {
                    JsonObject o = arr.add<JsonObject>();
                    o["ssid"] = n.ssid;
                    o["rssi"] = n.rssi;
                    o["secure"] = n.isSecure;
                    o["auth"] = n.authMode;
                }
                serializeJson(scanDoc, Serial);
                Serial.println();
                if (cloudClient.isEnabled()) {
                    cloudClient.pushTelemetry(scanDoc);
                }
            } else if (strcmp(commandType, "wifi_reset") == 0) {
                networkManager.resetWifiCredentials();
                Serial.println("[CMD_OK] Wi-Fi credentials reset to defaults");
            } else if (strcmp(commandType, "ota_flash") == 0 || strcmp(commandType, "ota_update") == 0 || strcmp(commandType, "ota_start") == 0) {
                const char* otaUrl = doc["url"] | "";
                const char* md5 = doc["md5"] | "";
                const char* ver = doc["version"] | "";
                Serial.printf("[OTA] Firmware update requested. URL: %s | MD5: %s | Ver: %s\n", otaUrl, md5, ver);
                if (strlen(otaUrl) > 0) {
                    if (!ac::ota::OtaManager::performOta(String(otaUrl), String(md5))) {
                        String errMsg = ac::ota::OtaManager::getLastError();
                        if (errMsg.length() == 0) {
                            errMsg = "OTA flash rejected or failed on controller";
                        }
                        Serial.printf("[OTA] Firmware update failed: %s\n", errMsg.c_str());
                        if (cloudClient.isEnabled()) {
                            JsonDocument errDoc;
                            errDoc["device_id"] = deviceId;
                            errDoc["ota_status"] = "failed";
                            errDoc["ota_error"] = errMsg;
                            errDoc["timestamp_ms"] = millis();
                            cloudClient.pushTelemetry(errDoc);
                        }
                    }
                } else if (strcmp(commandType, "ota_flash") == 0) {
                    String errMsg = "OTA flash command missing download URL";
                    Serial.println("[OTA] Error: Missing URL in ota_flash command");
                    if (cloudClient.isEnabled()) {
                        JsonDocument errDoc;
                        errDoc["device_id"] = deviceId;
                        errDoc["ota_status"] = "failed";
                        errDoc["ota_error"] = errMsg;
                        errDoc["timestamp_ms"] = millis();
                        cloudClient.pushTelemetry(errDoc);
                    }
                } else {
                    Serial.println("[OTA] Standby: Firmware staged. Awaiting binary stream pull.");
                }
            }
            return;
        }
    }

    // String commands
    String upper = cmd;
    upper.toUpperCase();

    if (upper.startsWith("SET_AC_STATE")) {
        int spaceIdx = cmd.indexOf(' ');
        if (spaceIdx > 0) {
            String args = cmd.substring(spaceIdx + 1);
            args.trim();
            std::vector<String> tokens;
            int start = 0;
            while (start < (int)args.length()) {
                int nextSpace = args.indexOf(' ', start);
                if (nextSpace == -1) {
                    tokens.push_back(args.substring(start));
                    break;
                }
                tokens.push_back(args.substring(start, nextSpace));
                start = nextSpace + 1;
                while (start < (int)args.length() && args.charAt(start) == ' ') start++;
            }
            if (tokens.size() >= 4) {
                String pwrStr = tokens[0];
                pwrStr.toLowerCase();
                bool pwr = (pwrStr == "1" || pwrStr == "true" || pwrStr == "on");
                int temp = tokens[1].toInt();
                String mode = tokens[2];
                mode.toLowerCase();
                String fan = tokens[3];
                fan.toLowerCase();

                if (temp < ac::control::AzureEssenceController::kMinTemperature ||
                    temp > ac::control::AzureEssenceController::kMaxTemperature) {
                    Serial.println("[CMD_ERR] Temperature must be between 16 and 31 C");
                    return;
                }
                if (mode != "cool" && mode != "dry" && mode != "fan" && mode != "auto") {
                    Serial.println("[CMD_ERR] Mode must be cool, dry, fan, or auto");
                    return;
                }
                if (fan != "auto" && fan != "med" && fan != "low" && fan != "high") {
                    Serial.println("[CMD_ERR] Fan speed must be auto, med, low, or high");
                    return;
                }

                digitalWrite(STATUS_LED_PIN, HIGH);
                if (acController.setState(pwr, (uint8_t)temp, mode, fan, "serial")) {
                    lastTxBlankingTime = millis();
                    safetyManager.recordPowerTransition(pwr, millis());
                    safetyManager.recordCommandSent(millis());
                    closedLoopFeedback.notifyCommandSent(acController.getState(), millis());
                    automationEngine.setUserManualPowerOff(!pwr);
                    digitalWrite(STATUS_LED_PIN, LOW);
                    Serial.printf("[CMD_OK] AC state set: Power=%d Temp=%d Mode=%s Fan=%s\n",
                                  pwr ? 1 : 0, temp, mode.c_str(), fan.c_str());
                } else {
                    digitalWrite(STATUS_LED_PIN, LOW);
                    Serial.println("[CMD_ERR] Failed to set AC state");
                }
            } else {
                Serial.println("[CMD_ERR] Usage: SET_AC_STATE <1|0> <temp> <mode> <fan>");
            }
        } else {
            Serial.println("[CMD_ERR] Usage: SET_AC_STATE <1|0> <temp> <mode> <fan>");
        }
    } else if (upper == "POWER_ON" || upper == "ON") {
        digitalWrite(STATUS_LED_PIN, HIGH);
        lastTxBlankingTime = millis();
        acController.setPower(true, "manual_cmd");
        safetyManager.recordPowerTransition(true, millis());
        closedLoopFeedback.notifyCommandSent(acController.getState(), millis());
        automationEngine.setUserManualPowerOff(false);
        digitalWrite(STATUS_LED_PIN, LOW);
        Serial.println("[CMD_OK] AC Power ON sent");
    } else if (upper == "POWER_OFF" || upper == "OFF") {
        digitalWrite(STATUS_LED_PIN, HIGH);
        lastTxBlankingTime = millis();
        acController.setPower(false, "manual_cmd");
        safetyManager.recordPowerTransition(false, millis());
        closedLoopFeedback.notifyCommandSent(acController.getState(), millis());
        automationEngine.setUserManualPowerOff(true);
        digitalWrite(STATUS_LED_PIN, LOW);
        Serial.println("[CMD_OK] AC Power OFF sent");
    } else if (upper.startsWith("SET_TEMP")) {
        int spaceIdx = upper.indexOf(' ');
        if (spaceIdx > 0) {
            int tempVal = upper.substring(spaceIdx + 1).toInt();
            if (tempVal >= ac::control::AzureEssenceController::kMinTemperature &&
                tempVal <= ac::control::AzureEssenceController::kMaxTemperature) {
                digitalWrite(STATUS_LED_PIN, HIGH);
                lastTxBlankingTime = millis();
                acController.setTemperature((uint8_t)tempVal, "serial");
                safetyManager.recordCommandSent(millis());
                closedLoopFeedback.notifyCommandSent(acController.getState(), millis());
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
                lastTxBlankingTime = millis();
                acController.setMode(m, "serial");
                safetyManager.recordCommandSent(millis());
                closedLoopFeedback.notifyCommandSent(acController.getState(), millis());
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
            lastTxBlankingTime = millis();
            acController.setFanSpeed(spd, "serial");
            safetyManager.recordCommandSent(millis());
            closedLoopFeedback.notifyCommandSent(acController.getState(), millis());
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
    } else if (upper == "GET_RADAR_DEBUG") {
        presenceSensor.printDebug(Serial);
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
    } else if (upper.startsWith("SET_PRESENCE_POLARITY")) {
        int spaceIdx = upper.indexOf(' ');
        if (spaceIdx > 0) {
            String pol = upper.substring(spaceIdx + 1);
            pol.trim();
            pol.toUpperCase();
            bool activeLow = (pol == "LOW" || pol == "ACTIVE_LOW" || pol == "TRUE" || pol == "1");
            presenceSensor.setActiveLow(activeLow);
            Serial.printf("[CMD_OK] Presence sensor polarity set to Active-%s\n", activeLow ? "LOW" : "HIGH");
        } else {
            Serial.println("[CMD_ERR] Usage: SET_PRESENCE_POLARITY <HIGH|LOW>");
        }
    } else if (upper.startsWith("SET_RADAR_GATES")) {
        int spaceIdx = upper.indexOf(' ');
        if (spaceIdx > 0) {
            String rest = upper.substring(spaceIdx + 1);
            rest.trim();
            int secondSpace = rest.indexOf(' ');
            if (secondSpace > 0) {
                float minM = rest.substring(0, secondSpace).toFloat();
                float maxM = rest.substring(secondSpace + 1).toFloat();
                presenceSensor.setDistanceGates(minM, maxM);
                Serial.printf("[CMD_OK] Radar gates set to %.2fm - %.2fm\n", minM, maxM);
            } else {
                Serial.println("[CMD_ERR] Usage: SET_RADAR_GATES <min_m> <max_m>");
            }
        } else {
            Serial.println("[CMD_ERR] Usage: SET_RADAR_GATES <min_m> <max_m>");
        }
    } else if (upper.startsWith("SET_RADAR_ENERGY")) {
        int spaceIdx = upper.indexOf(' ');
        if (spaceIdx > 0) {
            String rest = upper.substring(spaceIdx + 1);
            rest.trim();
            int secondSpace = rest.indexOf(' ');
            if (secondSpace > 0) {
                uint8_t moveE = (uint8_t)rest.substring(0, secondSpace).toInt();
                uint8_t statE = (uint8_t)rest.substring(secondSpace + 1).toInt();
                presenceSensor.setEnergyThresholds(moveE, statE);
                Serial.printf("[CMD_OK] Radar energy thresholds set to Move=%u%% Stat=%u%%\n", moveE, statE);
            } else {
                Serial.println("[CMD_ERR] Usage: SET_RADAR_ENERGY <min_move> <min_stat>");
            }
        } else {
            Serial.println("[CMD_ERR] Usage: SET_RADAR_ENERGY <min_move> <min_stat>");
        }
    } else if (upper.startsWith("SET_ABSENCE_TIMEOUT")) {
        int spaceIdx = upper.indexOf(' ');
        if (spaceIdx > 0) {
            uint32_t to = upper.substring(spaceIdx + 1).toInt();
            presenceSensor.setAbsenceTimeoutSeconds(to);
            Serial.printf("[CMD_OK] Absence timeout set to %u seconds\n", to);
        } else {
            Serial.println("[CMD_ERR] Usage: SET_ABSENCE_TIMEOUT <seconds>");
        }
    } else if (upper.startsWith("SET_CLOSED_LOOP")) {
        int spaceIdx = upper.indexOf(' ');
        if (spaceIdx > 0) {
            String val = upper.substring(spaceIdx + 1);
            val.trim();
            bool en = (val == "1" || val == "ON" || val == "TRUE");
            closedLoopFeedback.setEnabled(en);
            Serial.printf("[CMD_OK] Closed loop feedback %s\n", en ? "ENABLED" : "DISABLED");
        } else {
            Serial.println("[CMD_ERR] Usage: SET_CLOSED_LOOP <1|0>");
        }
    } else if (upper == "GET_CLOSED_LOOP") {
        JsonDocument clDoc;
        closedLoopFeedback.toJSON(clDoc);
        String clOut;
        serializeJson(clDoc, clOut);
        Serial.println(clOut);
    } else if (upper.startsWith("SET_COMFORT")) {
        int spaceIdx = upper.indexOf(' ');
        if (spaceIdx > 0) {
            String val = upper.substring(spaceIdx + 1);
            val.trim();
            bool en = (val == "1" || val == "ON" || val == "TRUE");
            automationEngine.setComfortIndexOptimization(en);
            Serial.printf("[CMD_OK] Comfort index optimization %s\n", en ? "ENABLED" : "DISABLED");
        } else {
            Serial.println("[CMD_ERR] Usage: SET_COMFORT <1|0>");
        }
    } else if (upper.startsWith("SET_NIGHT_CYCLE")) {
        int spaceIdx = upper.indexOf(' ');
        if (spaceIdx > 0) {
            String rest = upper.substring(spaceIdx + 1);
            rest.trim();
            int secondSpace = rest.indexOf(' ');
            bool en = false;
            float tgt = 27.0f;
            if (secondSpace > 0) {
                String enStr = rest.substring(0, secondSpace);
                en = (enStr == "1" || enStr == "ON" || enStr == "TRUE");
                tgt = rest.substring(secondSpace + 1).toFloat();
                if (tgt < 18.0f || tgt > 31.0f) tgt = 27.0f;
            } else {
                en = (rest == "1" || rest == "ON" || rest == "TRUE");
            }
            automationEngine.setNightCycle(en, tgt);
            Serial.printf("[CMD_OK] Night sleep cycle %s (Target: %.1fC)\n", en ? "ENABLED" : "DISABLED", tgt);
        } else {
            Serial.println("[CMD_ERR] Usage: SET_NIGHT_CYCLE <1|0> [target_temp]");
        }
    } else if (upper == "TEST_TX") {
        digitalWrite(STATUS_LED_PIN, HIGH);
        lastTxBlankingTime = millis();
        acController.getTransmitter().testPowerOn();
        safetyManager.recordPowerTransition(true, millis());
        digitalWrite(STATUS_LED_PIN, LOW);
        Serial.println("[CMD_OK] Transmitted Test Power ON frame (Cool, 25C, Auto Fan) with repeat");
    } else if (upper.startsWith("SET_WIFI")) {
        int spaceIdx = cmd.indexOf(' ');
        if (spaceIdx > 0) {
            String rest = cmd.substring(spaceIdx + 1);
            rest.trim();
            int secondSpace = rest.indexOf(' ');
            String ssid = "";
            String pass = "";
            if (secondSpace > 0) {
                ssid = rest.substring(0, secondSpace);
                pass = rest.substring(secondSpace + 1);
                pass.trim();
            } else {
                ssid = rest;
            }
            if (ssid.length() > 0) {
                networkManager.configureWifi(ssid, pass);
                Serial.printf("[CMD_OK] Configured Wi-Fi: %s\n", ssid.c_str());
            } else {
                Serial.println("[CMD_ERR] SSID cannot be empty");
            }
        } else {
            Serial.println("[CMD_ERR] Usage: SET_WIFI <ssid> [password]");
        }
    } else if (upper == "WIFI_SCAN") {
        std::vector<ac::network::ScannedNetwork> nets = networkManager.scanNetworks();
        Serial.printf("[WIFI_SCAN] Found %d networks:\n", (int)nets.size());
        for (const auto& n : nets) {
            Serial.printf("  - %s (%d dBm, %s)\n", n.ssid.c_str(), (int)n.rssi, n.authMode.c_str());
        }
    } else if (upper == "WIFI_RESET") {
        networkManager.resetWifiCredentials();
        Serial.println("[CMD_OK] Wi-Fi credentials cleared, AP started");
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
            uint32_t now = millis();
            uint32_t lastTx = acController.getTransmitter().getLastTxEndTime();
            if ((lastTxBlankingTime > 0 && (now - lastTxBlankingTime < 300)) ||
                (lastTx > 0 && (now - lastTx < 300))) {
                Serial.printf("[IR_BLANKING] Ignoring demodulated IR signal within %lu ms of transmission (self-loopback)\n",
                              (unsigned long)(now - (lastTxBlankingTime > lastTx ? lastTxBlankingTime : lastTx)));
                irReceiver.clearLastCapture();
            } else {
                const ac::ir::IRCaptureInfo& cap = irReceiver.getLastCapture();
                if (cap.hasAcState) {
                    acController.applyExternalState(cap.acState);
                    safetyManager.recordPowerTransition(cap.acState.power, millis());
                    safetyManager.recordCommandSent(millis());
                    automationEngine.setUserManualPowerOff(!cap.acState.power);
                    JsonDocument syncDoc;
                    syncDoc["device_id"] = deviceId;
                    syncDoc["power"] = cap.acState.power;
                    syncDoc["temperature"] = cap.acState.temperature;
                    syncDoc["mode"] = cap.acState.mode;
                    syncDoc["fan_speed"] = cap.acState.fanSpeed;
                    syncDoc["manual_power_off_override"] = !cap.acState.power;
                    syncDoc["source"] = "ir_remote";
                    syncDoc["timestamp_ms"] = millis();
                    serializeJson(syncDoc, Serial);
                    Serial.println();
                    if (cloudClient.isEnabled()) {
                        cloudClient.pushTelemetry(syncDoc);
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
                    cloudClient.pushTelemetry(irDoc);
                }
                irReceiver.clearLastCapture();
            }
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
        if (cloudClient.isEnabled()) {
            cloudClient.pushTelemetry(doc);
        }
    }
#endif

    // 4. Update Local Automation Engine
    automationEngine.update();

    // 5. Update Energy Monitoring Integration (accumulate continuously, refresh AC load model at 1Hz)
    static uint32_t lastEnergyModelUpdate = 0;
    uint32_t currentNow = millis();
#if HAS_CURRENT_SENSOR
    energyMonitor.sampleAdcCurrent(CURRENT_SENSOR_PIN, 30.0f);
#endif
    energyMonitor.update(currentNow);
    float currentAmbientTemp = dhtDriver.getLatestReading().valid ? dhtDriver.getLatestReading().temperature_c : 25.0f;
    closedLoopFeedback.update(currentNow, currentAmbientTemp);
    if (currentNow - lastEnergyModelUpdate >= 1000) {
        lastEnergyModelUpdate = currentNow;
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
        cloudDoc["firmware_version"] = FIRMWARE_VERSION;
        cloudDoc["build"] = __DATE__;
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
        cloudDoc["distance_m"] = presenceSensor.getDistanceM();
        cloudDoc["target_count"] = presenceSensor.getTargetCount();
        cloudDoc["motion_state"] = presenceSensor.getMotionState();
        cloudDoc["radar_uart_active"] = presenceSensor.isUartActive();
        cloudDoc["radar_uart_bytes"] = presenceSensor.getUartBytesReceived();
        cloudDoc["presence_tier"] = automationEngine.getPresenceTierStr();
        cloudDoc["sleep_stage"] = automationEngine.getSleepStageStr();
        cloudDoc["sleep_enabled"] = automationEngine.isCircadianSleepEnabled();
        cloudDoc["thermal_breach"] = automationEngine.isThermalBreachActive();
        cloudDoc["thermal_breach_delta"] = automationEngine.getThermalBreachDelta();
        cloudDoc["eco_drift_enabled"] = automationEngine.isEcoDriftEnabled();
        cloudDoc["psychrometric_enabled"] = automationEngine.isPsychrometricEnabled();
        cloudDoc["auto_enabled"] = automationEngine.isEnabled();
        cloudDoc["presence_detection_enabled"] = automationEngine.isPresenceDetectionEnabled();
        cloudDoc["moving_energy"] = presenceSensor.getMovingEnergy();
        cloudDoc["stationary_energy"] = presenceSensor.getStationaryEnergy();
        cloudDoc["radar_min_dist"] = presenceSensor.getMinDistanceM();
        cloudDoc["radar_max_dist"] = presenceSensor.getMaxDistanceM();
        cloudDoc["absence_timeout_s"] = presenceSensor.getAbsenceTimeoutSeconds();
        cloudDoc["radar_in_zone"] = presenceSensor.isRawInZoneDetected();
        cloudDoc["power_watts"] = energyMonitor.getPowerWatts();
        cloudDoc["power_tier"] = energyMonitor.getPowerTierStr();
        cloudDoc["closed_loop_status"] = closedLoopFeedback.getStatusStr();
        cloudDoc["closed_loop_retries"] = closedLoopFeedback.getRetryCount();
        cloudDoc["comfort_opt"] = automationEngine.isComfortIndexOptimization();
        cloudDoc["voltage"] = energyMonitor.getVoltage();
        cloudDoc["current"] = energyMonitor.getCurrent();
        cloudDoc["energy_kwh_today"] = energyMonitor.getEnergyKwhToday();
        cloudDoc["energy_units_today"] = energyMonitor.getEnergyKwhToday();
        cloudDoc["estimated_cost_today"] = energyMonitor.getEstimatedCostToday();
        cloudDoc["tariff_rate"] = energyMonitor.getTariff();
        cloudDoc["ir_tx_installed"] = HAS_IR_TRANSMITTER;
        cloudDoc["ir_rx_installed"] = true;
        cloudDoc["local_ip"] = networkManager.getIPAddress();
        cloudDoc["wifi_ssid"] = networkManager.getSSID();
        cloudDoc["wifi_rssi"] = networkManager.getRSSI();
        cloudDoc["is_ap"] = networkManager.isAPMode();
        cloudDoc["night_cycle_enabled"] = automationEngine.isNightCycleEnabled();
        cloudDoc["night_cycle_stage"] = automationEngine.getNightCycleStageStr();
        cloudDoc["night_cycle_stage_id"] = (uint8_t)automationEngine.getNightCycleStage();
        cloudDoc["night_cycle_target_temp"] = automationEngine.getNightCycleTargetTemp();
        cloudDoc["night_cycle_remaining_s"] = automationEngine.getNightCycleStageRemainingSec();
        cloudDoc["manual_power_off_override"] = automationEngine.isUserManualPowerOff();
        cloudDoc["thermal_stall_desync"] = closedLoopFeedback.isThermalStallDesync();
        cloudDoc["thermal_stall_suspended"] = energyMonitor.isThermalStallSuspended();
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
            cloudClient.pushTelemetry(cloudDoc);
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
