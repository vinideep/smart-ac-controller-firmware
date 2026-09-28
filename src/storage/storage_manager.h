#pragma once

#include <Arduino.h>
#include "../ac/ac_state.h"

namespace ac::storage {

struct RadarStorageConfig {
    float minDistanceM = 0.2f;
    float maxDistanceM = 4.5f;
    uint8_t minMovingEnergy = 15;
    uint8_t minStationaryEnergy = 15;
    uint32_t absenceTimeoutS = 600; // 10 minutes default
};

struct ClosedLoopStorageConfig {
    bool enabled = true;
    uint32_t verifyTimeoutS = 60;
    uint8_t maxRetries = 2;
    float standbyThresholdW = 20.0f;
    float compressorThresholdW = 250.0f;
};

class StorageManager {
public:
    static bool saveAcState(const control::ACState& state);
    static bool loadAcState(control::ACState& state);

    static bool saveRadarConfig(const RadarStorageConfig& cfg);
    static bool loadRadarConfig(RadarStorageConfig& cfg);

    static bool saveClosedLoopConfig(const ClosedLoopStorageConfig& cfg);
    static bool loadClosedLoopConfig(ClosedLoopStorageConfig& cfg);

    static bool saveWifiCredentials(const String& ssid, const String& password);
    static bool loadWifiCredentials(String& ssid, String& password);
    static bool clearWifiCredentials();
};

} // namespace ac::storage
