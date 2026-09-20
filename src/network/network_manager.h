#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>

namespace ac::network {

class NetworkManager {
public:
    NetworkManager();

    void begin(const char* ssid = nullptr, const char* password = nullptr, const char* hostname = "smart-ac");
    void update();

    bool isConnected() const;
    String getIPAddress() const;
    int8_t getRSSI() const;
    bool isAPMode() const { return _isAPMode; }
    String getHostname() const { return _hostname; }

    void startAP(const char* apSsid = "AzureEssence-SmartAC", const char* apPass = "azure1234");

private:
    String _ssid;
    String _password;
    String _hostname;
    bool _isAPMode = false;
    uint32_t _lastReconnectAttempt = 0;
    uint32_t _connectStartTime = 0;
    bool _connecting = false;
};

} // namespace ac::network
