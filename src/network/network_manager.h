#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <vector>
#include <algorithm>

namespace ac::network {

struct ScannedNetwork {
    String ssid;
    int32_t rssi;
    bool isSecure;
    String authMode;
};

class NetworkManager {
public:
    NetworkManager();

    void begin(const char* fallbackSsid = nullptr, const char* fallbackPass = nullptr, const char* hostname = "smart-ac");
    void update();

    bool isConnected() const;
    String getIPAddress() const;
    int8_t getRSSI() const;
    bool isAPMode() const { return _isAPMode; }
    String getHostname() const { return _hostname; }
    String getSSID() const { return _ssid; }

    void startAP(const char* apSsid = "AzureEssence-SmartAC", const char* apPass = "");
    void stopAP();

    std::vector<ScannedNetwork> scanNetworks();
    bool configureWifi(const String& ssid, const String& password);
    void resetWifiCredentials();

private:
    String _ssid;
    String _password;
    String _hostname;
    String _fallbackSsid;
    String _fallbackPass;
    bool _isAPMode = false;
    bool _dnsRunning = false;
    DNSServer _dnsServer;
    uint32_t _lastReconnectAttempt = 0;
    uint32_t _connectStartTime = 0;
    bool _connecting = false;
    uint32_t _stopApAt = 0;
    uint32_t _pendingConnectAt = 0;
    bool _pendingConnect = false;
    uint32_t _disconnectedSince = 0;
};

} // namespace ac::network
