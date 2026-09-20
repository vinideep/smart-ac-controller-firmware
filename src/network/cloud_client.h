#pragma once
/**
 * @file cloud_client.h
 * @brief HTTP cloud bridge — pushes telemetry to backend, polls for commands.
 *
 * Allows the ESP32 to operate fully standalone (USB charger, no Mac needed).
 * The backend URL and device token are set in config/config.h before flashing.
 *
 * Communication model:
 *   ESP32 → POST /api/ingest        (telemetry push, every 3s)
 *   ESP32 ← GET  /api/commands/pending  (command poll, every 2s)
 */

#include <Arduino.h>
#include <ArduinoJson.h>

namespace ac {
namespace cloud {

/**
 * CloudClient
 *
 * Manages periodic HTTP communication with the cloud backend.
 * Must be called from loop() via update().
 * Commands received from the backend are queued internally;
 * call hasCommand() / dequeueCommand() to consume them.
 */
class CloudClient {
public:
    /**
     * @param backendUrl  Full HTTPS URL of backend, e.g. "https://smart-ac.up.railway.app"
     * @param deviceToken Shared secret matching DEVICE_TOKEN on backend env
     */
    CloudClient(const char* backendUrl, const char* deviceToken);

    /**
     * Call once in setup() after Wi-Fi is connected.
     */
    void begin();

    /**
     * Call every loop() iteration.
     * Fires telemetry push and command poll on their respective intervals.
     * @param telemetryDoc  JsonDocument containing current sensor + AC state
     */
    void update(const JsonDocument& telemetryDoc);

    /** Returns true if at least one command is waiting to be executed. */
    bool hasCommand() const;

    /**
     * Returns and removes the oldest pending command string.
     * Returns empty string if no command is queued.
     * Command strings match the same format as serial commands:
     *   "POWER_ON", "POWER_OFF", "SET_TEMP 25", "SET_FAN AUTO", etc.
     */
    String dequeueCommand();

    /** Returns true if backend URL is configured (non-empty). */
    bool isEnabled() const;

    /** Returns timestamp_ms of last successful telemetry push (0 = never). */
    unsigned long lastPushMs() const { return _lastPushMs; }

    /** Returns timestamp_ms of last successful command poll (0 = never). */
    unsigned long lastPollMs() const { return _lastPollMs; }

private:
    const char* _backendUrl;
    const char* _deviceToken;

    unsigned long _lastTelemetryTimer = 0;
    unsigned long _lastPollTimer     = 0;
    unsigned long _lastPushMs        = 0;
    unsigned long _lastPollMs        = 0;

    // Simple fixed-size command queue (ring buffer)
    static constexpr uint8_t kQueueSize = 8;
    String  _cmdQueue[kQueueSize];
    uint8_t _qHead = 0;
    uint8_t _qTail = 0;
    uint8_t _qCount = 0;

    void enqueue(const String& cmd);
    bool pushTelemetry(const JsonDocument& doc);
    bool pollCommands();
};

} // namespace cloud
} // namespace ac
