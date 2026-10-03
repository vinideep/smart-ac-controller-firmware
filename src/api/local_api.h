#pragma once

#include <Arduino.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include "../ac/ac_controller.h"
#include "../sensors/dht_sensor.h"
#include "../sensors/presence_sensor.h"
#include "../safety/safety_manager.h"
#include "../network/network_manager.h"

namespace ac::api {

class LocalAPIServer {
public:
    LocalAPIServer(control::ACController& ac,
                   DHTDriver& dht,
                   sensors::PresenceSensorDriver& presence,
                   safety::SafetyManager& safety,
                   network::NetworkManager& network,
                   const String& deviceId);

    void begin(uint16_t port = 80);
    void update();

private:
    WebServer _server;
    control::ACController& _ac;
    DHTDriver& _dht;
    sensors::PresenceSensorDriver& _presence;
    safety::SafetyManager& _safety;
    network::NetworkManager& _network;
    const String& _deviceId;

    void setupRoutes();
    void sendCORS();
    void handleRoot();
    void handleStatus();
    void handleTelemetry();
    void handleGetState();
    void handleSetState();
    void handleSetPower();
    void handleSetTemperature();
    void handleSetFan();
    void handleSetMode();
    void handleWifiScan();
    void handleWifiConfigure();
    void handleWifiStatus();
};

} // namespace ac::api
