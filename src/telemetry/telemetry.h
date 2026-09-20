#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>
#include "../sensors/dht_sensor.h"
#include "../sensors/energy_monitor.h"

class TelemetryManager {
public:
    static void printReadingJSON(const DHTReading& reading, const String& deviceId);
    static void printEnergyJSON(const ac::sensors::EnergyReading& energy, const String& deviceId);
    static void printDeviceInfo(const String& deviceId);
};
