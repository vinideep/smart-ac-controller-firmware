#include "storage_manager.h"

#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
#include <Preferences.h>
#endif

namespace ac::storage {

bool StorageManager::saveAcState(const control::ACState& state) {
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
    Preferences prefs;
    if (!prefs.begin("ac_state", false)) return false;
    prefs.putBool("power", state.power);
    prefs.putUChar("temp", state.temperature);
    prefs.putString("mode", state.mode);
    prefs.putString("fan", state.fanSpeed);
    prefs.putBool("swing", state.swing);
    prefs.putBool("sleep", state.sleep);
    prefs.end();
    return true;
#else
    return false;
#endif
}

bool StorageManager::loadAcState(control::ACState& state) {
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
    Preferences prefs;
    if (!prefs.begin("ac_state", true)) return false;
    state.power = prefs.getBool("power", false);
    state.temperature = prefs.getUChar("temp", 25);
    state.mode = prefs.getString("mode", "cool");
    state.fanSpeed = prefs.getString("fan", "auto");
    state.swing = prefs.getBool("swing", false);
    state.sleep = prefs.getBool("sleep", false);
    state.source = "nvs_restored";
    state.timestamp = millis();
    prefs.end();
    return true;
#else
    return false;
#endif
}

bool StorageManager::saveRadarConfig(const RadarStorageConfig& cfg) {
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
    Preferences prefs;
    if (!prefs.begin("radar_cfg", false)) return false;
    prefs.putFloat("min_m", cfg.minDistanceM);
    prefs.putFloat("max_m", cfg.maxDistanceM);
    prefs.putUChar("move_e", cfg.minMovingEnergy);
    prefs.putUChar("stat_e", cfg.minStationaryEnergy);
    prefs.putUInt("abs_s", cfg.absenceTimeoutS);
    prefs.end();
    return true;
#else
    return false;
#endif
}

bool StorageManager::loadRadarConfig(RadarStorageConfig& cfg) {
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
    Preferences prefs;
    if (!prefs.begin("radar_cfg", true)) return false;
    cfg.minDistanceM = prefs.getFloat("min_m", cfg.minDistanceM);
    cfg.maxDistanceM = prefs.getFloat("max_m", cfg.maxDistanceM);
    cfg.minMovingEnergy = prefs.getUChar("move_e", cfg.minMovingEnergy);
    cfg.minStationaryEnergy = prefs.getUChar("stat_e", cfg.minStationaryEnergy);
    cfg.absenceTimeoutS = prefs.getUInt("abs_s", cfg.absenceTimeoutS);
    prefs.end();
    return true;
#else
    return false;
#endif
}

bool StorageManager::saveClosedLoopConfig(const ClosedLoopStorageConfig& cfg) {
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
    Preferences prefs;
    if (!prefs.begin("cl_cfg", false)) return false;
    prefs.putBool("en", cfg.enabled);
    prefs.putUInt("to_s", cfg.verifyTimeoutS);
    prefs.putUChar("retries", cfg.maxRetries);
    prefs.putFloat("stby_w", cfg.standbyThresholdW);
    prefs.putFloat("comp_w", cfg.compressorThresholdW);
    prefs.end();
    return true;
#else
    return false;
#endif
}

bool StorageManager::loadClosedLoopConfig(ClosedLoopStorageConfig& cfg) {
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
    Preferences prefs;
    if (!prefs.begin("cl_cfg", true)) return false;
    cfg.enabled = prefs.getBool("en", cfg.enabled);
    cfg.verifyTimeoutS = prefs.getUInt("to_s", cfg.verifyTimeoutS);
    cfg.maxRetries = prefs.getUChar("retries", cfg.maxRetries);
    cfg.standbyThresholdW = prefs.getFloat("stby_w", cfg.standbyThresholdW);
    cfg.compressorThresholdW = prefs.getFloat("comp_w", cfg.compressorThresholdW);
    prefs.end();
    return true;
#else
    return false;
#endif
}

bool StorageManager::saveWifiCredentials(const String& ssid, const String& password) {
    if (ssid.length() == 0) return false;
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
    Preferences prefs;
    if (!prefs.begin("wifi_cfg", false)) return false;
    prefs.putString("ssid", ssid);
    prefs.putString("pass", password);
    prefs.end();
    return true;
#else
    return false;
#endif
}

bool StorageManager::loadWifiCredentials(String& ssid, String& password) {
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
    Preferences prefs;
    if (!prefs.begin("wifi_cfg", true)) return false;
    ssid = prefs.getString("ssid", "");
    password = prefs.getString("pass", "");
    prefs.end();
    return (ssid.length() > 0);
#else
    return false;
#endif
}

bool StorageManager::clearWifiCredentials() {
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
    Preferences prefs;
    if (!prefs.begin("wifi_cfg", false)) return false;
    prefs.clear();
    prefs.end();
    return true;
#else
    return false;
#endif
}

} // namespace ac::storage
