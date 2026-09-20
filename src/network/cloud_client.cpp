#include "cloud_client.h"
#include "../config/config.h"
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <WiFi.h>

namespace ac {
namespace cloud {

// ─── Constructor ────────────────────────────────────────────────────────────
CloudClient::CloudClient(const char* backendUrl, const char* deviceToken)
    : _backendUrl(backendUrl), _deviceToken(deviceToken) {}

void CloudClient::begin() {
    if (!isEnabled()) {
        Serial.println("[CLOUD] Disabled — BACKEND_URL is empty. Running in local-only mode.");
        return;
    }
    Serial.printf("[CLOUD] Cloud client enabled → %s\n", _backendUrl);
}

bool CloudClient::isEnabled() const {
    return _backendUrl != nullptr && strlen(_backendUrl) > 4; // must have "http"
}

// ─── Main update loop ────────────────────────────────────────────────────────
void CloudClient::update(const JsonDocument& telemetryDoc) {
    if (!isEnabled()) return;
    if (WiFi.status() != WL_CONNECTED) return;

    unsigned long now = millis();

    // Push telemetry every CLOUD_TELEMETRY_INTERVAL_MS
    if (now - _lastTelemetryTimer >= CLOUD_TELEMETRY_INTERVAL_MS) {
        _lastTelemetryTimer = now;
        pushTelemetry(telemetryDoc);
    }

    // Poll for commands every CLOUD_POLL_INTERVAL_MS
    if (now - _lastPollTimer >= CLOUD_POLL_INTERVAL_MS) {
        _lastPollTimer = now;
        pollCommands();
    }
}

// ─── Telemetry push ──────────────────────────────────────────────────────────
bool CloudClient::pushTelemetry(const JsonDocument& doc) {
    String body;
    serializeJson(doc, body);

    HTTPClient http;
    WiFiClientSecure secureClient;
    String url = String(_backendUrl) + "/api/ingest";

    if (url.startsWith("https://")) {
        secureClient.setInsecure();
        secureClient.setHandshakeTimeout(10);
        http.begin(secureClient, url);
    } else {
        http.begin(url);
    }

    http.addHeader("Content-Type", "application/json");
    http.addHeader("Authorization", String("Bearer ") + _deviceToken);
    http.setTimeout(5000);

    int code = http.POST(body);
    bool ok = (code == 200 || code == 201);

    if (ok) {
        _lastPushMs = millis();
        Serial.println("[CLOUD] Telemetry pushed to server (HTTP OK)");
    } else if (code > 0) {
        Serial.printf("[CLOUD_WARN] Telemetry push failed: HTTP %d\n", code);
    } else {
        Serial.printf("[CLOUD_WARN] Telemetry push error: %s\n", http.errorToString(code).c_str());
    }

    http.end();
    return ok;
}

// ─── Command poll ─────────────────────────────────────────────────────────────
bool CloudClient::pollCommands() {
    HTTPClient http;
    WiFiClientSecure secureClient;
    String url = String(_backendUrl) + "/api/commands/pending";

    if (url.startsWith("https://")) {
        secureClient.setInsecure();
        secureClient.setHandshakeTimeout(10);
        http.begin(secureClient, url);
    } else {
        http.begin(url);
    }

    http.addHeader("Authorization", String("Bearer ") + _deviceToken);
    http.setTimeout(4000);

    int code = http.GET();

    if (code == 200) {
        String payload = http.getString();
        http.end();

        // Parse response: {"commands":["POWER_ON","SET_TEMP 25"],"count":2}
        JsonDocument resp;
        if (deserializeJson(resp, payload) == DeserializationError::Ok) {
            JsonArray cmds = resp["commands"].as<JsonArray>();
            int count = 0;
            for (JsonVariant v : cmds) {
                String cmdStr = v.as<String>();
                cmdStr.trim();
                if (cmdStr.length() > 0) {
                    enqueue(cmdStr);
                    count++;
                }
            }
            if (count > 0) {
                Serial.printf("[CLOUD] Received %d command(s) from backend\n", count);
                _lastPollMs = millis();
            }
        }
        return true;
    }

    if (code > 0) {
        Serial.printf("[CLOUD_WARN] Command poll failed: HTTP %d\n", code);
    }
    http.end();
    return false;
}

// ─── Command queue ────────────────────────────────────────────────────────────
void CloudClient::enqueue(const String& cmd) {
    if (_qCount >= kQueueSize) {
        // Drop oldest to make room
        _qHead = (_qHead + 1) % kQueueSize;
        _qCount--;
    }
    _cmdQueue[_qTail] = cmd;
    _qTail = (_qTail + 1) % kQueueSize;
    _qCount++;
}

bool CloudClient::hasCommand() const {
    return _qCount > 0;
}

String CloudClient::dequeueCommand() {
    if (_qCount == 0) return "";
    String cmd = _cmdQueue[_qHead];
    _qHead = (_qHead + 1) % kQueueSize;
    _qCount--;
    return cmd;
}

} // namespace cloud
} // namespace ac
