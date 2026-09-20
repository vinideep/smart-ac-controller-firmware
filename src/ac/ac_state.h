#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>

namespace ac::control {

/**
 * @brief Representation of complete AC operational state.
 * Air conditioners transmit the entire operational state in every command frame.
 */
struct ACState {
    bool power = false;
    uint8_t temperature = 25;       // Range: 16°C - 31°C
    String mode = "cool";           // "cool", "fan", "dry", "auto"
    String fanSpeed = "auto";       // "auto", "low", "med", "high"
    bool swing = false;
    bool sleep = false;
    String source = "system";       // "serial", "ai", "telemetry", "user"
    uint32_t timestamp = 0;

    template <typename T>
    void serializeTo(T& obj) const {
        obj["power"] = power;
        obj["temperature"] = temperature;
        obj["mode"] = mode;
        obj["fan_speed"] = fanSpeed;
        obj["swing"] = swing;
        obj["sleep"] = sleep;
        obj["source"] = source;
        obj["timestamp_ms"] = timestamp;
    }

    void toJSON(JsonDocument& doc) const {
        serializeTo(doc);
    }

    String toJSONString() const {
        JsonDocument doc;
        toJSON(doc);
        String out;
        serializeJson(doc, out);
        return out;
    }
};

} // namespace ac::control
