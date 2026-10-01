#include "network_manager.h"
#include "../storage/storage_manager.h"
#include <esp_wifi.h>

namespace ac::network {

NetworkManager::NetworkManager() {}

void NetworkManager::begin(const char* fallbackSsid, const char* fallbackPass, const char* hostname) {
    _hostname = hostname ? String(hostname) : "smart-ac";
    _fallbackSsid = fallbackSsid ? String(fallbackSsid) : "";
    _fallbackPass = fallbackPass ? String(fallbackPass) : "";

    String savedSsid = "";
    String savedPass = "";
    bool hasSaved = storage::StorageManager::loadWifiCredentials(savedSsid, savedPass);

    if (hasSaved && savedSsid.length() > 0) {
        _ssid = savedSsid;
        _password = savedPass;
        Serial.printf("[WIFI] Loaded saved credentials from NVS for SSID: %s\n", _ssid.c_str());
    } else if (_fallbackSsid.length() > 0) {
        _ssid = _fallbackSsid;
        _password = _fallbackPass;
        Serial.printf("[WIFI] Using default macro credentials for SSID: %s\n", _ssid.c_str());
    } else {
        _ssid = "";
        _password = "";
        Serial.println("[WIFI] No saved or default credentials found; starting standalone AP mode.");
    }

    if (_ssid.length() > 0) {
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

    IPAddress apIP(192, 168, 4, 1);
    IPAddress gateway(192, 168, 4, 1);
    IPAddress subnet(255, 255, 255, 0);
    WiFi.softAPConfig(apIP, gateway, subnet);

    bool res = (apPass && strlen(apPass) >= 8) ? 
        WiFi.softAP(apSsid, apPass) : 
        WiFi.softAP(apSsid);

    _isAPMode = true;
    if (res) {
        Serial.printf("[WIFI_AP] Access Point started: %s (IP: %s)\n", 
                      apSsid, WiFi.softAPIP().toString().c_str());

        _dnsServer.stop();
        if (_dnsServer.start(53, "*", apIP)) {
            _dnsRunning = true;
            Serial.println("[DNS] Captive Portal DNS server started on port 53");
        } else {
            Serial.println("[DNS_ERR] Failed to start Captive Portal DNS server");
        }

        if (MDNS.begin(_hostname.c_str())) {
            MDNS.addService("http", "tcp", 80);
            Serial.printf("[MDNS] Responder started: http://%s.local\n", _hostname.c_str());
        }
    } else {
        Serial.println("[WIFI_AP_ERR] Failed to start Access Point");
    }
}

void NetworkManager::stopAP() {
    if (_dnsRunning) {
        _dnsServer.stop();
        _dnsRunning = false;
        Serial.println("[DNS] Captive Portal DNS server stopped");
    }
    if (_isAPMode) {
        _isAPMode = false;
        WiFi.softAPdisconnect(true);
        if (WiFi.status() == WL_CONNECTED) {
            WiFi.mode(WIFI_STA);
        }
        Serial.println("[WIFI_AP] Access Point stopped");
    }
}

void NetworkManager::update() {
    if (_dnsRunning) {
        _dnsServer.processNextRequest();
    }

    if (_stopApAt > 0 && millis() >= _stopApAt) {
        _stopApAt = 0;
        stopAP();
    }

    // Deferred connect after HTTP response finishes flushing
    if (_pendingConnect && millis() >= _pendingConnectAt) {
        _pendingConnect = false;
        if (WiFi.getMode() != WIFI_AP_STA && WiFi.getMode() != WIFI_STA) {
            WiFi.mode(WIFI_AP_STA);
        }
        WiFi.disconnect(false, false);
        delay(50);
        WiFi.begin(_ssid.c_str(), _password.c_str());
        _connecting = true;
        _connectStartTime = millis();
        _lastReconnectAttempt = millis();
        _disconnectedSince = 0;
        Serial.printf("[WIFI] Initiating background connection to: %s ...\n", _ssid.c_str());
    }

    if (WiFi.status() == WL_CONNECTED) {
        _disconnectedSince = 0;
        if (_connecting) {
            _connecting = false;
            Serial.printf("[WIFI] Connected! IP Address: %s (RSSI: %d dBm)\n", 
                          WiFi.localIP().toString().c_str(), WiFi.RSSI());
            if (MDNS.begin(_hostname.c_str())) {
                MDNS.addService("http", "tcp", 80);
                Serial.printf("[MDNS] Responder started: http://%s.local\n", _hostname.c_str());
            }
            if (_isAPMode && _stopApAt == 0) {
                // Keep AP alive briefly so web clients receive their success response
                _stopApAt = millis() + 5000;
            }
        }
    } else {
        uint32_t now = millis();

        // 1. Check for initial or configuration connection timeout
        if (_connecting && (now - _connectStartTime > 25000)) {
            _connecting = false;
            Serial.printf("[WIFI_WARN] Connection attempt to '%s' timed out (>25s).\n", _ssid.c_str());
            if (!_isAPMode) {
                Serial.println("[WIFI] Launching Captive Portal AP fallback...");
                startAP();
            }
        }

        // 2. Check for sustained disconnect while running in STA mode
        if (!_isAPMode && !_connecting) {
            if (_disconnectedSince == 0) {
                _disconnectedSince = now;
            } else if (now - _disconnectedSince > 25000) {
                Serial.println("[WIFI_WARN] Wi-Fi lost for >25s. Starting Captive Portal AP fallback...");
                startAP();
                _disconnectedSince = 0;
            }
        }

        // 3. Periodic reconnect: safely attempt reconnection every 25s if credentials exist
        if (_ssid.length() > 0 && !_connecting && (now - _lastReconnectAttempt > 25000)) {
            _lastReconnectAttempt = now;
            if (_isAPMode) {
                if (WiFi.getMode() != WIFI_AP_STA) WiFi.mode(WIFI_AP_STA);
            }
            Serial.printf("[WIFI] Attempting connection to SSID '%s'...\n", _ssid.c_str());
            WiFi.begin(_ssid.c_str(), _password.c_str());
            _connecting = true;
            _connectStartTime = now;
        }
    }
}

std::vector<ScannedNetwork> NetworkManager::scanNetworks() {
    std::vector<ScannedNetwork> results;

    wifi_mode_t currentMode = WiFi.getMode();
    if (currentMode == WIFI_MODE_NULL || currentMode == WIFI_MODE_AP) {
        WiFi.mode(WIFI_AP_STA);
        delay(100);
    }

    Serial.println("[WIFI] Scanning for available networks...");
    int16_t n = WiFi.scanNetworks(false, false);
    Serial.printf("[WIFI] Scan finished, found %d networks\n", n);

    if (n > 0) {
        for (int16_t i = 0; i < n; ++i) {
            String ssid = WiFi.SSID(i);
            if (ssid.length() == 0) continue;

            bool exists = false;
            for (auto& item : results) {
                if (item.ssid == ssid) {
                    exists = true;
                    if (WiFi.RSSI(i) > item.rssi) {
                        item.rssi = WiFi.RSSI(i);
                    }
                    break;
                }
            }
            if (!exists) {
                ScannedNetwork net;
                net.ssid = ssid;
                net.rssi = WiFi.RSSI(i);
                wifi_auth_mode_t auth = WiFi.encryptionType(i);
                net.isSecure = (auth != WIFI_AUTH_OPEN);
                switch (auth) {
                    case WIFI_AUTH_OPEN: net.authMode = "Open"; break;
                    case WIFI_AUTH_WEP: net.authMode = "WEP"; break;
                    case WIFI_AUTH_WPA_PSK: net.authMode = "WPA"; break;
                    case WIFI_AUTH_WPA2_PSK: net.authMode = "WPA2"; break;
                    case WIFI_AUTH_WPA_WPA2_PSK: net.authMode = "WPA/WPA2"; break;
                    case WIFI_AUTH_WPA2_ENTERPRISE: net.authMode = "WPA2-Enterprise"; break;
                    case WIFI_AUTH_WPA3_PSK: net.authMode = "WPA3"; break;
                    case WIFI_AUTH_WPA2_WPA3_PSK: net.authMode = "WPA2/WPA3"; break;
                    default: net.authMode = "Secure"; break;
                }
                results.push_back(net);
            }
        }
        WiFi.scanDelete();

        // Sort networks descending by signal strength (strongest first)
        std::sort(results.begin(), results.end(), [](const ScannedNetwork& a, const ScannedNetwork& b) {
            return a.rssi > b.rssi;
        });
    } else if (n == 0) {
        WiFi.scanDelete();
    }

    return results;
}

bool NetworkManager::configureWifi(const String& ssid, const String& password) {
    if (ssid.length() == 0) return false;

    _ssid = ssid;
    _password = password;

    bool saved = storage::StorageManager::saveWifiCredentials(_ssid, _password);
    Serial.printf("[WIFI] Saved credentials for '%s' to NVS: %s\n", _ssid.c_str(), saved ? "OK" : "FAILED");

    // Defer the radio switch by 150ms so current HTTP request response can be sent cleanly
    _pendingConnect = true;
    _pendingConnectAt = millis() + 150;
    return true;
}

void NetworkManager::resetWifiCredentials() {
    storage::StorageManager::clearWifiCredentials();
    _ssid = "";
    _password = "";
    WiFi.disconnect(true, true);
    startAP();
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
