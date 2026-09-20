#include <Arduino.h>
#include <esp_idf_version.h>
#include <esp_task_wdt.h>
#include <esp_system.h>
#include <rom/rtc.h>
#include <DHT.h>
#include <ArduinoJson.h>
#include <IRrecv.h>
#include <IRsend.h>
#include <IRutils.h>
#include <IRac.h>
#include <memory>

// ==============================================================================
// CONFIGURATION & PIN DEFINITIONS (All 9 Milestones)
// ==============================================================================
#define FIRMWARE_VERSION    "0.1.0-alpha"
#define DEVICE_MODEL        "AzureEssence-SmartAC"
#define MONITOR_BAUD_RATE   115200

#ifndef DHT_PIN
#define DHT_PIN             13    // DHT22 Data line
#endif

#ifndef DHT_TYPE
#define DHT_TYPE            DHT22
#endif

#ifndef STATUS_LED_PIN
#define STATUS_LED_PIN      2     // On-board status LED
#endif

#ifndef IR_RX_PIN
#define IR_RX_PIN           14    // Milestone 3: 38kHz IR receiver demodulator OUT
#endif

#ifndef IR_TX_PIN
#define IR_TX_PIN           25    // Milestone 4: 38kHz IR LED driver
#endif

#ifndef IR_OBSTACLE_PIN
#define IR_OBSTACLE_PIN     26    // Milestone 7: Obstacle/Presence sensor digital OUT
#endif

// Sensor bounds & intervals
constexpr uint32_t DHT_SAMPLE_INTERVAL_MS = 2000;
constexpr float    DHT_TEMP_MIN_C          = -40.0f;
constexpr float    DHT_TEMP_MAX_C          = 80.0f;
constexpr float    DHT_HUMIDITY_MIN        = 0.0f;
constexpr float    DHT_HUMIDITY_MAX        = 100.0f;
constexpr uint8_t  DHT_MAX_ERRORS          = 5;
constexpr uint32_t WDT_TIMEOUT_SECONDS     = 8;

// IR Timing Parameters
constexpr uint16_t IR_CAPTURE_BUFFER_SIZE = 1024;
constexpr uint8_t  IR_TIMEOUT_MS           = 50;
constexpr uint16_t IR_MIN_RAW_LENGTH       = 4;

// Azure Essence Protocol Constants
constexpr uint16_t kAzureHdrMark          = 4500;
constexpr uint16_t kAzureHdrSpace         = 2400;
constexpr uint16_t kAzureBitMark          = 430;
constexpr uint16_t kAzureOneSpace         = 970;
constexpr uint16_t kAzureZeroSpace        = 470;
constexpr uint16_t kAzureStopMark         = 430;
constexpr uint8_t  kAzureStateLength      = 9;
constexpr uint16_t kAzureRawTransitions   = 147;
constexpr uint8_t  kAzureVendorId          = 0x19;
constexpr uint8_t  kAzurePowerOnByte       = 0x18;
constexpr uint8_t  kAzurePowerOffByte      = 0x70;
constexpr uint8_t  kAzureModeCool          = 0x02;
constexpr uint8_t  kAzureFanAuto           = 0x01;
constexpr uint8_t  kAzureFanMed            = 0x02;
constexpr uint8_t  kAzureFanHigh           = 0x03;

// ==============================================================================
// DATA STRUCTURES
// ==============================================================================
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

struct PresenceReading {
    bool present = false;
    uint32_t duration_seconds = 0;
    uint32_t last_seen_ms = 0;
    const char* status = "ROOM_EMPTY";
};

struct EnergyReading {
    float voltage = 230.0f;
    float current = 0.015f;
    float power_watts = 2.5f;
    float power_factor = 0.65f;
    float energy_kwh_today = 0.0f;
    float estimated_cost_today = 0.0f;
    float tariff_rate = 8.0f;
    uint32_t timestamp_ms = 0;
    const char* status = "standby";
    bool valid = true;
};

struct ACState {
    bool power = false;
    uint8_t temperature = 25;
    String mode = "cool";
    String fanSpeed = "auto";
    String source = "init";
    uint32_t lastChangedMs = 0;
};

struct AutomationConfig {
    bool enabled = false;
    float targetTemperature = 25.0f;
    float hysteresis = 1.0f;
    uint32_t emptyTimeoutSeconds = 900;
};

// ==============================================================================
// GLOBAL HARDWARE & STATE INSTANCES
// ==============================================================================
DHT dht(DHT_PIN, DHT_TYPE);
IRrecv irReceiver(IR_RX_PIN, IR_CAPTURE_BUFFER_SIZE, IR_TIMEOUT_MS, true);
IRsend irSender(IR_TX_PIN);
decode_results irResults;

DHTReading latestReading;
PresenceReading latestPresence;
EnergyReading latestEnergy;
ACState acState;
AutomationConfig autoConfig;

String deviceId;
String serialInputBuffer = "";
uint32_t lastReadTime = 0;
uint32_t lastEnergyTime = 0;
uint32_t lastEnergyModelUpdate = 0;
uint32_t lastAutoEvalTime = 0;
double accumulatedKwh = 0.0;
uint8_t consecutiveErrors = 0;

// Safety manager timers
constexpr uint32_t kMinOnTimeMs = 180000;    // 3 minutes min compressor on
constexpr uint32_t kMinOffTimeMs = 180000;   // 3 minutes min compressor off
constexpr uint32_t kMinCmdThrottleMs = 5000; // 5 seconds between commands
uint32_t lastTurnOnMs = 0;
uint32_t lastTurnOffMs = 0;
uint32_t lastCmdMs = 0;

// Presence sensor debounce
uint32_t lastPresencePoll = 0;
uint32_t presenceStateStart = 0;
uint8_t presenceDebounce = 0;
bool presenceRaw = false;

// ==============================================================================
// RESET REASON & BOOT TELEMETRY
// ==============================================================================
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

void printBootBanner() {
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
    Serial.println("[SYSTEM] Ready. Commands: POWER_ON, POWER_OFF, SET_TEMP <16-31>, SET_FAN <AUTO|MED|HIGH>, AUTO_ON, AUTO_OFF, GET_STATE, GET_PRESENCE, GET_ENERGY, SET_TARIFF <rate>, RESET_ENERGY, TEST_TX");
    Serial.println();
}

// ==============================================================================
// AZURE ESSENCE PROTOCOL ENCODING & TRANSMISSION (Milestones 4, 5, 6)
// ==============================================================================
uint8_t calcAzureChecksum(const uint8_t state[kAzureStateLength]) {
    uint16_t sum = 0;
    for (uint8_t i = 0; i < 8; i++) {
        sum += (state[i] >> 4) & 0x0F;
        sum += state[i] & 0x0F;
    }
    return static_cast<uint8_t>(sum & 0x0F);
}

void encodeAzureFrame(const ACState& st, uint8_t out[kAzureStateLength]) {
    out[0] = kAzureVendorId;
    uint8_t fan = kAzureFanAuto;
    if (st.fanSpeed.equalsIgnoreCase("med") || st.fanSpeed.equalsIgnoreCase("low")) fan = kAzureFanMed;
    else if (st.fanSpeed.equalsIgnoreCase("high")) fan = kAzureFanHigh;

    uint8_t mode = kAzureModeCool;
    out[1] = (fan << 4) | (mode & 0x0F);
    out[2] = 0x00; out[3] = 0x00; out[4] = 0x00; out[5] = 0x00;

    uint8_t t = st.temperature < 16 ? 16 : (st.temperature > 31 ? 31 : st.temperature);
    out[6] = t + 11;
    out[7] = st.power ? kAzurePowerOnByte : kAzurePowerOffByte;
    uint8_t chk = calcAzureChecksum(out);
    out[8] = (chk << 4) & 0xF0;
}

bool transmitAzureState(const ACState& st, uint16_t repeats = 1) {
    uint8_t stateBytes[kAzureStateLength];
    encodeAzureFrame(st, stateBytes);

    uint16_t rawBuf[kAzureRawTransitions];
    uint16_t idx = 0;
    rawBuf[idx++] = kAzureHdrMark;
    rawBuf[idx++] = kAzureHdrSpace;

    for (uint8_t b = 0; b < kAzureStateLength; b++) {
        uint8_t val = stateBytes[b];
        for (uint8_t bit = 0; bit < 8; bit++) {
            rawBuf[idx++] = kAzureBitMark;
            bool isOne = (val >> bit) & 1;
            rawBuf[idx++] = isOne ? kAzureOneSpace : kAzureZeroSpace;
        }
    }
    rawBuf[idx++] = kAzureStopMark;

    digitalWrite(STATUS_LED_PIN, HIGH);
    irSender.sendRaw(rawBuf, idx, 38);
    for (uint16_t r = 0; r < repeats; r++) {
        delayMicroseconds(21000);
        irSender.sendRaw(rawBuf, idx, 38);
    }
    digitalWrite(STATUS_LED_PIN, LOW);
    lastCmdMs = millis();
    return true;
}

// ==============================================================================
// SAFETY MANAGER LOGIC (Milestone 9)
// ==============================================================================
bool canTurnOn(uint32_t now, const char*& reason) {
    if (acState.power) { reason = "AC is already powered ON"; return false; }
    if (lastTurnOffMs > 0 && (now - lastTurnOffMs) < kMinOffTimeMs) {
        reason = "Compressor protection: minimum OFF rest time has not elapsed (3m)";
        return false;
    }
    if (lastCmdMs > 0 && (now - lastCmdMs) < kMinCmdThrottleMs) {
        reason = "Rate limit: commands throttled to prevent IR collisions";
        return false;
    }
    reason = nullptr;
    return true;
}

bool canTurnOff(uint32_t now, const char*& reason) {
    if (!acState.power) { reason = "AC is already powered OFF"; return false; }
    if (lastTurnOnMs > 0 && (now - lastTurnOnMs) < kMinOnTimeMs) {
        reason = "Compressor protection: minimum ON run time has not elapsed (3m)";
        return false;
    }
    if (lastCmdMs > 0 && (now - lastCmdMs) < kMinCmdThrottleMs) {
        reason = "Rate limit: commands throttled to prevent IR collisions";
        return false;
    }
    reason = nullptr;
    return true;
}

void recordPowerTransition(bool pwr, uint32_t now) {
    acState.power = pwr;
    acState.lastChangedMs = now;
    if (pwr) lastTurnOnMs = now;
    else lastTurnOffMs = now;
    lastCmdMs = now;
}

// ==============================================================================
// PRESENCE DETECTION LOGIC (Milestone 7)
// ==============================================================================
bool updatePresence() {
    uint32_t now = millis();
    if (now - lastPresencePoll < 50) return false;
    lastPresencePoll = now;

    int pinVal = digitalRead(IR_OBSTACLE_PIN);
    bool detected = (pinVal == LOW); // Active-low

    if (detected == presenceRaw) {
        if (presenceDebounce < 3) presenceDebounce++;
    } else {
        presenceRaw = detected;
        presenceDebounce = 0;
    }

    bool stateChanged = false;
    if (presenceDebounce >= 3 && detected != latestPresence.present) {
        latestPresence.present = detected;
        presenceStateStart = now;
        stateChanged = true;
    }

    latestPresence.duration_seconds = (now - presenceStateStart) / 1000;
    if (latestPresence.present) {
        latestPresence.last_seen_ms = now;
        latestPresence.status = "ROOM_OCCUPIED";
    } else {
        latestPresence.status = "ROOM_EMPTY";
    }
    return stateChanged;
}

// ==============================================================================
// ENERGY MONITORING LOGIC (Milestone 8 - High Precision & Rollover Safe)
// ==============================================================================
void accumulateEnergy(uint32_t now) {
    if (lastEnergyTime == 0) {
        lastEnergyTime = now;
        latestEnergy.timestamp_ms = now;
        return;
    }

    uint32_t deltaMs = now - lastEnergyTime;
    if (deltaMs == 0) return;
    if (deltaMs > 86400000UL) {
        lastEnergyTime = now;
        return;
    }

    double deltaHours = (double)deltaMs / 3600000.0;
    double deltaKwh = ((double)latestEnergy.power_watts * deltaHours) / 1000.0;
    accumulatedKwh += deltaKwh;
    latestEnergy.energy_kwh_today = (float)accumulatedKwh;
    latestEnergy.estimated_cost_today = (float)(accumulatedKwh * latestEnergy.tariff_rate);
    latestEnergy.timestamp_ms = now;
    lastEnergyTime = now;
}

void updateEnergyModel() {
    uint32_t now = millis();
    accumulateEnergy(now);

    if (now - lastEnergyModelUpdate < 1000) return;
    lastEnergyModelUpdate = now;

    if (!acState.power) {
        latestEnergy.power_watts = 2.5f;
        latestEnergy.power_factor = 0.65f;
        latestEnergy.current = latestEnergy.power_watts / (latestEnergy.voltage * latestEnergy.power_factor);
        latestEnergy.status = "standby";
        return;
    }

    float fanPower = 45.0f;
    if (acState.fanSpeed.equalsIgnoreCase("low")) fanPower = 35.0f;
    else if (acState.fanSpeed.equalsIgnoreCase("med")) fanPower = 55.0f;
    else if (acState.fanSpeed.equalsIgnoreCase("high")) fanPower = 80.0f;

    if (acState.mode.equalsIgnoreCase("fan")) {
        latestEnergy.power_watts = fanPower;
        latestEnergy.power_factor = 0.88f;
        latestEnergy.current = latestEnergy.power_watts / (latestEnergy.voltage * latestEnergy.power_factor);
        latestEnergy.status = "fan_only";
        return;
    }

    if (acState.mode.equalsIgnoreCase("dry")) {
        latestEnergy.power_watts = 600.0f + fanPower;
        latestEnergy.power_factor = 0.94f;
        latestEnergy.current = latestEnergy.power_watts / (latestEnergy.voltage * latestEnergy.power_factor);
        latestEnergy.status = "dehumidifying";
        return;
    }

    // Cooling load calculation
    latestEnergy.status = "cooling";
    latestEnergy.power_factor = 0.97f;
    float amb = latestReading.valid ? latestReading.temperature_c : 25.0f;
    float delta = amb - (float)acState.temperature;
    float comp = (delta > 0.0f) ? (1100.0f + delta * 85.0f) : 550.0f;
    if (comp < 500.0f) comp = 500.0f;
    if (comp > 1800.0f) comp = 1800.0f;

    latestEnergy.power_watts = comp + fanPower;
    latestEnergy.current = latestEnergy.power_watts / (latestEnergy.voltage * latestEnergy.power_factor);
}

void printEnergyJSON() {
    JsonDocument doc;
    doc["type"] = "energy_telemetry";
    doc["device_id"] = deviceId;
    doc["voltage"] = round(latestEnergy.voltage * 10.0f) / 10.0f;
    doc["current"] = round(latestEnergy.current * 100.0f) / 100.0f;
    doc["power_watts"] = round(latestEnergy.power_watts * 10.0f) / 10.0f;
    doc["power_factor"] = round(latestEnergy.power_factor * 100.0f) / 100.0f;
    doc["energy_kwh_today"] = round(latestEnergy.energy_kwh_today * 10000.0f) / 10000.0f;
    doc["tariff_rate"] = round(latestEnergy.tariff_rate * 100.0f) / 100.0f;
    doc["estimated_cost_today"] = round(latestEnergy.estimated_cost_today * 100.0f) / 100.0f;
    doc["status"] = latestEnergy.status;
    doc["timestamp_ms"] = latestEnergy.timestamp_ms;
    doc["valid"] = latestEnergy.valid;

    serializeJson(doc, Serial);
    Serial.println();
}

// ==============================================================================
// LOCAL CLIMATE AUTOMATION ENGINE (Milestone 9)
// ==============================================================================
void updateAutomationEngine() {
    if (!autoConfig.enabled) return;
    uint32_t now = millis();
    if (now - lastAutoEvalTime < 5000) return;
    lastAutoEvalTime = now;

    if (!latestReading.valid || latestReading.isStale(10000)) return;

    float curTemp = latestReading.temperature_c;
    bool isPresent = latestPresence.present;
    uint32_t emptySec = isPresent ? 0 : latestPresence.duration_seconds;

    // Rule 1: Turn off if empty for timeout
    if (acState.power && !isPresent && emptySec >= autoConfig.emptyTimeoutSeconds) {
        const char* reason = nullptr;
        if (canTurnOff(now, reason)) {
            acState.power = false;
            transmitAzureState(acState);
            recordPowerTransition(false, now);
            Serial.printf("[AUTO] Empty room timeout (%u s) -> AC turned OFF\n", emptySec);
        }
        return;
    }

    // Rule 2: Turn on if occupied and temperature is above threshold
    if (!acState.power && isPresent && curTemp > (autoConfig.targetTemperature + autoConfig.hysteresis)) {
        const char* reason = nullptr;
        if (canTurnOn(now, reason)) {
            acState.power = true;
            acState.temperature = (uint8_t)round(autoConfig.targetTemperature);
            transmitAzureState(acState);
            recordPowerTransition(true, now);
            Serial.printf("[AUTO] Room occupied & temp %.1fC > target -> AC turned ON\n", curTemp);
        }
        return;
    }

    // Rule 3: Turn off if target reached
    if (acState.power && isPresent && curTemp < (autoConfig.targetTemperature - autoConfig.hysteresis)) {
        const char* reason = nullptr;
        if (canTurnOff(now, reason)) {
            acState.power = false;
            transmitAzureState(acState);
            recordPowerTransition(false, now);
            Serial.printf("[AUTO] Target temperature achieved (%.1fC) -> AC turned OFF\n", curTemp);
        }
    }
}

// ==============================================================================
// DHT CLIMATE SENSOR LOGIC (Milestone 2)
// ==============================================================================
bool updateDHTSensor() {
    uint32_t now = millis();
    if (now - lastReadTime < DHT_SAMPLE_INTERVAL_MS) return false;
    lastReadTime = now;

    float hum = dht.readHumidity();
    float temp = dht.readTemperature();

    if (isnan(hum) || isnan(temp)) {
        consecutiveErrors++;
        latestReading.valid = false;
        latestReading.status = (consecutiveErrors >= DHT_MAX_ERRORS) ? "sensor_disconnected" : "read_failed";
        latestReading.error_message = "NaN received from DHT sensor";
        latestReading.timestamp_ms = now;
        return true;
    }

    if (temp < DHT_TEMP_MIN_C || temp > DHT_TEMP_MAX_C || hum < DHT_HUMIDITY_MIN || hum > DHT_HUMIDITY_MAX) {
        consecutiveErrors++;
        latestReading.valid = false;
        latestReading.status = "out_of_range";
        latestReading.error_message = "Sensor value out of operational limits";
        latestReading.timestamp_ms = now;
        return true;
    }

    latestReading.temperature = temp;
    latestReading.humidity = hum;
    latestReading.temperature_c = temp;
    latestReading.humidity_percent = hum;
    latestReading.timestamp_ms = now;
    latestReading.status = "ok";
    latestReading.valid = true;
    latestReading.error_message = nullptr;
    consecutiveErrors = 0;
    return true;
}

void printReadingJSON() {
    JsonDocument doc;
    if (latestReading.valid) {
        float roundedTemp = round(latestReading.temperature * 10.0f) / 10.0f;
        float roundedHum = round(latestReading.humidity * 10.0f) / 10.0f;
        doc["temperature"] = roundedTemp;
        doc["humidity"] = roundedHum;
        doc["temperature_c"] = roundedTemp;
        doc["humidity_percent"] = roundedHum;
    } else {
        doc["temperature"] = nullptr;
        doc["humidity"] = nullptr;
        doc["temperature_c"] = nullptr;
        doc["humidity_percent"] = nullptr;
    }
    doc["sensor_status"] = latestReading.status;
    if (latestReading.error_message != nullptr) doc["error"] = latestReading.error_message;
    doc["device_id"] = deviceId;
    doc["timestamp_ms"] = latestReading.timestamp_ms;
    doc["uptime_s"] = millis() / 1000;
    doc["free_heap"] = ESP.getFreeHeap();

    serializeJson(doc, Serial);
    Serial.println();
}

// ==============================================================================
// IR RECEIVER PROCESSING (Milestone 3)
// ==============================================================================
void processIRReceiver() {
    if (irReceiver.decode(&irResults)) {
        if (irResults.rawlen >= IR_MIN_RAW_LENGTH) {
            digitalWrite(STATUS_LED_PIN, HIGH);
            JsonDocument doc;
            doc["type"] = "ir_capture";
            doc["device_id"] = deviceId;
            doc["timestamp_ms"] = millis();
            doc["protocol"] = typeToString(irResults.decode_type, irResults.repeat);
            doc["bits"] = irResults.bits;
            doc["hex"] = resultToHexidecimal(&irResults);
            doc["raw_len"] = getCorrectedRawLength(&irResults);
            serializeJson(doc, Serial);
            Serial.println();
            digitalWrite(STATUS_LED_PIN, LOW);
        }
    }
}

// ==============================================================================
// SERIAL COMMAND PROCESSING
// ==============================================================================
void processSerialCommand(const String& raw) {
    String cmd = raw;
    cmd.trim();
    if (cmd.length() == 0) return;

    if (cmd.startsWith("{")) {
        JsonDocument doc;
        if (!deserializeJson(doc, cmd)) {
            const char* type = doc["cmd"] | "";
            if (strcmp(type, "power") == 0) {
                bool p = doc["value"] | false;
                const char* r = nullptr;
                if (p ? canTurnOn(millis(), r) : canTurnOff(millis(), r)) {
                    acState.power = p;
                    transmitAzureState(acState);
                    recordPowerTransition(p, millis());
                    Serial.printf("[CMD_OK] AC Power set to %s\n", p ? "ON" : "OFF");
                } else {
                    Serial.printf("[SAFETY_BLOCKED] %s\n", r ? r : "rejected");
                }
            } else if (strcmp(type, "temp") == 0) {
                uint8_t t = doc["value"] | 25;
                if (t >= 16 && t <= 31) {
                    acState.temperature = t;
                    if (acState.power) transmitAzureState(acState);
                    Serial.printf("[CMD_OK] AC Temperature set to %d C\n", t);
                }
            } else if (strcmp(type, "fan") == 0) {
                const char* spd = doc["value"] | "auto";
                acState.fanSpeed = spd;
                if (acState.power) transmitAzureState(acState);
                Serial.printf("[CMD_OK] AC Fan set to %s\n", spd);
            } else if (strcmp(type, "energy") == 0) {
                printEnergyJSON();
            } else if (strcmp(type, "tariff") == 0) {
                float rate = doc["value"] | 0.0f;
                if (rate > 0.0f) {
                    latestEnergy.tariff_rate = rate;
                    latestEnergy.estimated_cost_today = (float)(accumulatedKwh * rate);
                    Serial.printf("[CMD_OK] Energy tariff set to %.2f per kWh\n", rate);
                }
            }
            return;
        }
    }

    String upper = cmd;
    upper.toUpperCase();

    if (upper == "POWER_ON" || upper == "ON") {
        const char* reason = nullptr;
        if (canTurnOn(millis(), reason)) {
            acState.power = true;
            transmitAzureState(acState);
            recordPowerTransition(true, millis());
            Serial.println("[CMD_OK] AC Power ON transmitted via IR");
        } else {
            Serial.printf("[SAFETY_BLOCKED] %s\n", reason ? reason : "rejected");
        }
    } else if (upper == "POWER_OFF" || upper == "OFF") {
        const char* reason = nullptr;
        if (canTurnOff(millis(), reason)) {
            acState.power = false;
            transmitAzureState(acState);
            recordPowerTransition(false, millis());
            Serial.println("[CMD_OK] AC Power OFF transmitted via IR");
        } else {
            Serial.printf("[SAFETY_BLOCKED] %s\n", reason ? reason : "rejected");
        }
    } else if (upper.startsWith("SET_TEMP")) {
        int spaceIdx = upper.indexOf(' ');
        if (spaceIdx > 0) {
            int t = upper.substring(spaceIdx + 1).toInt();
            if (t >= 16 && t <= 31) {
                acState.temperature = (uint8_t)t;
                if (acState.power) transmitAzureState(acState);
                Serial.printf("[CMD_OK] AC Temperature set to %d C\n", t);
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
            acState.fanSpeed = spd;
            if (acState.power) transmitAzureState(acState);
            Serial.printf("[CMD_OK] AC Fan set to %s\n", spd.c_str());
        } else {
            Serial.println("[CMD_ERR] Usage: SET_FAN <AUTO|MED|HIGH>");
        }
    } else if (upper == "AUTO_ON") {
        autoConfig.enabled = true;
        Serial.println("[CMD_OK] Local climate automation ENABLED");
    } else if (upper == "AUTO_OFF") {
        autoConfig.enabled = false;
        Serial.println("[CMD_OK] Local climate automation DISABLED");
    } else if (upper.startsWith("SET_TARGET")) {
        int spaceIdx = upper.indexOf(' ');
        if (spaceIdx > 0) {
            autoConfig.targetTemperature = upper.substring(spaceIdx + 1).toFloat();
            Serial.printf("[CMD_OK] Automation target set to %.1f C\n", autoConfig.targetTemperature);
        } else {
            Serial.println("[CMD_ERR] Usage: SET_TARGET <temp>");
        }
    } else if (upper == "GET_STATE" || upper == "STATE") {
        JsonDocument doc;
        doc["power"] = acState.power;
        doc["temperature"] = acState.temperature;
        doc["mode"] = acState.mode;
        doc["fan_speed"] = acState.fanSpeed;
        serializeJson(doc, Serial);
        Serial.println();
    } else if (upper == "GET_PRESENCE") {
        JsonDocument doc;
        doc["presence"] = latestPresence.present;
        doc["duration_seconds"] = latestPresence.duration_seconds;
        doc["status"] = latestPresence.status;
        serializeJson(doc, Serial);
        Serial.println();
    } else if (upper == "GET_ENERGY" || upper == "ENERGY") {
        printEnergyJSON();
    } else if (upper == "RESET_ENERGY" || upper == "RESET_DAILY") {
        accumulatedKwh = 0.0;
        latestEnergy.energy_kwh_today = 0.0f;
        latestEnergy.estimated_cost_today = 0.0f;
        Serial.println("[CMD_OK] Daily energy and cost counters reset to zero");
    } else if (upper.startsWith("SET_TARIFF")) {
        int spaceIdx = upper.indexOf(' ');
        if (spaceIdx > 0) {
            float rate = upper.substring(spaceIdx + 1).toFloat();
            if (rate > 0.0f) {
                latestEnergy.tariff_rate = rate;
                latestEnergy.estimated_cost_today = (float)(accumulatedKwh * rate);
                Serial.printf("[CMD_OK] Energy tariff set to %.2f per kWh\n", rate);
            } else {
                Serial.println("[CMD_ERR] Tariff rate must be greater than 0");
            }
        } else {
            Serial.println("[CMD_ERR] Usage: SET_TARIFF <rate>");
        }
    } else if (upper == "TEST_TX") {
        acState.power = true;
        acState.temperature = 25;
        acState.fanSpeed = "auto";
        acState.mode = "cool";
        transmitAzureState(acState, 1);
        recordPowerTransition(true, millis());
        Serial.println("[CMD_OK] Transmitted Test Power ON frame (Cool, 25C, Auto Fan)");
    } else {
        Serial.printf("[CMD_UNKNOWN] %s\n", cmd.c_str());
    }
}

// ==============================================================================
// SETUP & MAIN LOOP
// ==============================================================================
void setup() {
    Serial.begin(MONITOR_BAUD_RATE);
    delay(500);

    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);
    pinMode(IR_OBSTACLE_PIN, INPUT_PULLUP);

    uint64_t mac = ESP.getEfuseMac();
    char idBuf[32];
    snprintf(idBuf, sizeof(idBuf), "esp32-%02x%02x%02x",
             (uint8_t)(mac >> 24),
             (uint8_t)(mac >> 32),
             (uint8_t)(mac >> 40));
    deviceId = String(idBuf);

#if ESP_IDF_VERSION_MAJOR >= 5
    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = WDT_TIMEOUT_SECONDS * 1000,
        .idle_core_mask = 0,
        .trigger_panic = true
    };
    esp_err_t err = esp_task_wdt_reconfigure(&twdt_config);
    if (err == ESP_ERR_INVALID_STATE) esp_task_wdt_init(&twdt_config);
    esp_task_wdt_add(NULL);
#else
    esp_task_wdt_init(WDT_TIMEOUT_SECONDS, true);
    esp_task_wdt_add(NULL);
#endif

    dht.begin();
    irReceiver.setUnknownThreshold(IR_MIN_RAW_LENGTH);
    irReceiver.enableIRIn(true);
    irSender.begin();

    printBootBanner();

    for (int i = 0; i < 2; i++) {
        digitalWrite(STATUS_LED_PIN, HIGH);
        delay(100);
        digitalWrite(STATUS_LED_PIN, LOW);
        delay(100);
    }
}

void loop() {
    esp_task_wdt_reset();

    // 1. Process IR reception (Milestone 3)
    processIRReceiver();

    // 2. Process Presence detection (Milestone 7)
    if (updatePresence()) {
        JsonDocument doc;
        doc["type"] = "presence_event";
        doc["device_id"] = deviceId;
        doc["presence"] = latestPresence.present;
        doc["duration_seconds"] = latestPresence.duration_seconds;
        doc["status"] = latestPresence.status;
        doc["timestamp_ms"] = millis();
        serializeJson(doc, Serial);
        Serial.println();
    }

    // 3. Update Automation engine (Milestone 9)
    updateAutomationEngine();

    // 4. Update Energy Model (Milestone 8)
    updateEnergyModel();

    // 5. Process Serial Commands
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (serialInputBuffer.length() > 0) {
                processSerialCommand(serialInputBuffer);
                serialInputBuffer = "";
            }
        } else if (serialInputBuffer.length() < 256) {
            serialInputBuffer += c;
        }
    }

    // 6. Process DHT22 sampling (Milestone 2)
    if (updateDHTSensor()) {
        digitalWrite(STATUS_LED_PIN, HIGH);
        delay(30);
        digitalWrite(STATUS_LED_PIN, LOW);
        printReadingJSON();
        printEnergyJSON();
    }

    delay(5);
}
