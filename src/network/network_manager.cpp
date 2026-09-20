#include "network_manager.h"
#include <esp_wifi.h>

namespace ac::network {

NetworkManager::NetworkManager() {}

void NetworkManager::begin(const char* ssid, const char* password, const char* hostname) {
    _hostname = hostname ? String(hostname) : "smart-ac";

    if (ssid && strlen(ssid) > 0) {
        _ssid = String(ssid);
        _password = password ? String(password) : "";
        WiFi.persistent(false);
        WiFi.disconnect(true, true);
        delay(100);
        WiFi.mode(WIFI_STA);
        WiFi.setSleep(false);
        esp_wifi_set_ps(WIFI_PS_NONE);
        WiFi.setHostname(_hostname.c_str());
        WiFi.setAutoReconnect(true);
        WiFi.begin(_ssid.c_str(), _password.c_str());
        _connecting = true;
        _connectStartTime = millis();
        _lastReconnectAttempt = millis();
        Serial.printf("[WIFI] Connecting to SSID: %s ...\n", _ssid.c_str());
    } else {
        startAP();
    }
}

void NetworkManager::startAP(const char* apSsid, const char* apPass) {
    if (_ssid.length() > 0) {
        WiFi.mode(WIFI_AP_STA);
    } else {
        WiFi.mode(WIFI_AP);
    }

    bool res = (apPass && strlen(apPass) >= 8) ? 
        WiFi.softAP(apSsid, apPass) : 
        WiFi.softAP(apSsid);

    _isAPMode = true;
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
    if (WiFi.status() == WL_CONNECTED) {
        if (_connecting || _isAPMode) {
            _connecting = false;
            _isAPMode = false;
            Serial.printf("[WIFI] Connected! IP Address: %s (RSSI: %d dBm)\n", 
                          WiFi.localIP().toString().c_str(), WiFi.RSSI());
            if (MDNS.begin(_hostname.c_str())) {
                MDNS.addService("http", "tcp", 80);
                Serial.printf("[MDNS] Responder started: http://%s.local\n", _hostname.c_str());
            }
        }
    } else {
        uint32_t now = millis();
        if (_ssid.length() > 0 && (now - _lastReconnectAttempt > 8000)) {
            _lastReconnectAttempt = now;
            if (_connecting && (now - _connectStartTime > 25000) && !_isAPMode) {
                Serial.println("[WIFI_WARN] Initial connection slow, launching concurrent AP fallback...");
                startAP();
            } else {
                Serial.printf("[WIFI] Re-attempting connection to %s...\n", _ssid.c_str());
                WiFi.begin(_ssid.c_str(), _password.c_str());
            }
        }
    }
}

bool NetworkManager::isConnected() const {
    return _isAPMode || (WiFi.status() == WL_CONNECTED);
}

String NetworkManager::getIPAddress() const {
    if (WiFi.status() == WL_CONNECTED) return WiFi.localIP().toString();
    if (_isAPMode) return WiFi.softAPIP().toString();
    return "0.0.0.0";
}

int8_t NetworkManager::getRSSI() const {
    return (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : 0;
}

} // namespace ac::network
