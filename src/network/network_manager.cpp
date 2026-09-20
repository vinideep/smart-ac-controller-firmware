#include "network_manager.h"

namespace ac::network {

NetworkManager::NetworkManager() {}

void NetworkManager::begin(const char* ssid, const char* password, const char* hostname) {
    _hostname = hostname ? String(hostname) : "smart-ac";

    if (ssid && strlen(ssid) > 0) {
        _ssid = String(ssid);
        _password = password ? String(password) : "";
        WiFi.mode(WIFI_STA);
        WiFi.setHostname(_hostname.c_str());
        WiFi.begin(_ssid.c_str(), _password.c_str());
        _connecting = true;
        _connectStartTime = millis();
        Serial.printf("[WIFI] Connecting to SSID: %s ...\n", _ssid.c_str());
    } else {
        startAP();
    }
}

void NetworkManager::startAP(const char* apSsid, const char* apPass) {
    WiFi.mode(WIFI_AP);
    bool res = (apPass && strlen(apPass) >= 8) ? 
        WiFi.softAP(apSsid, apPass) : 
        WiFi.softAP(apSsid);

    _isAPMode = true;
    _connecting = false;
    if (res) {
        Serial.printf("[WIFI_AP] Access Point started: %s (IP: %s)\n", 
                      apSsid, WiFi.softAPIP().toString().c_str());
        if (MDNS.begin(_hostname.c_str())) {
            MDNS.addService("http", "tcp", 80);
            Serial.printf("[MDNS] Responder started: http://%s.local\n", _hostname.c_str());
        }
    } else {
        Serial.println("[WIFI_AP_ERR] Failed to start Access Point");
    }
}

void NetworkManager::update() {
    if (_isAPMode) return;

    if (_connecting) {
        if (WiFi.status() == WL_CONNECTED) {
            _connecting = false;
            Serial.printf("[WIFI] Connected! IP Address: %s (RSSI: %d dBm)\n", 
                          WiFi.localIP().toString().c_str(), WiFi.RSSI());
            if (MDNS.begin(_hostname.c_str())) {
                MDNS.addService("http", "tcp", 80);
                Serial.printf("[MDNS] Responder started: http://%s.local\n", _hostname.c_str());
            }
        } else if (millis() - _connectStartTime > 15000) {
            // Wi-Fi connection timed out after 15s -> fallback to AP mode so user is never locked out
            Serial.println("[WIFI_TIMEOUT] Unable to connect to Wi-Fi. Launching Fallback AP...");
            startAP();
        }
    } else if (WiFi.status() != WL_CONNECTED) {
        // Lost connection -> non-blocking reconnect every 10 seconds
        uint32_t now = millis();
        if (now - _lastReconnectAttempt > 10000) {
            _lastReconnectAttempt = now;
            Serial.println("[WIFI_RECONNECT] Attempting Wi-Fi reconnection...");
            WiFi.reconnect();
        }
    }
}

bool NetworkManager::isConnected() const {
    return _isAPMode || (WiFi.status() == WL_CONNECTED);
}

String NetworkManager::getIPAddress() const {
    if (_isAPMode) return WiFi.softAPIP().toString();
    if (WiFi.status() == WL_CONNECTED) return WiFi.localIP().toString();
    return "0.0.0.0";
}

int8_t NetworkManager::getRSSI() const {
    return (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : 0;
}

} // namespace ac::network
