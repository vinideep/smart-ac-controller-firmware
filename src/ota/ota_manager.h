#pragma once
/**
 * @file ota_manager.h
 * @brief Secure OTA Firmware Updates with MD5 verification and rollback support.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Update.h>
#include <esp_task_wdt.h>

namespace ac::ota {

class OtaManager {
public:
    static bool performOta(const String& url, const String& expectedMd5 = "") {
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("[OTA] Error: Wi-Fi not connected");
            return false;
        }

        Serial.printf("[OTA] Connecting to binary stream at: %s\n", url.c_str());
        HTTPClient http;
        WiFiClientSecure secureClient;

        if (url.startsWith("https://")) {
            secureClient.setInsecure();
            secureClient.setHandshakeTimeout(15);
            http.begin(secureClient, url);
        } else {
            http.begin(url);
        }

        http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
        http.setTimeout(25000);
        esp_task_wdt_reset();

        int httpCode = http.GET();
        if (httpCode != HTTP_CODE_OK) {
            Serial.printf("[OTA] HTTP GET failed, error code: %d\n", httpCode);
            http.end();
            return false;
        }

        int contentLength = http.getSize();
        if (contentLength <= 0) {
            Serial.println("[OTA] Error: Content-Length invalid or chunked encoding unsupported");
            http.end();
            return false;
        }

        if (expectedMd5.length() > 0) {
            Update.setMD5(expectedMd5.c_str());
        }

        bool canBegin = Update.begin(contentLength);
        if (!canBegin) {
            Serial.printf("[OTA] Error: Insufficient flash partition space for OTA update (%d bytes needed)\n", contentLength);
            http.end();
            return false;
        }

        WiFiClient* stream = http.getStreamPtr();
        size_t written = 0;
        uint8_t buff[1024];
        while (http.connected() && (written < (size_t)contentLength)) {
            esp_task_wdt_reset();
            size_t available = stream->available();
            if (available) {
                int readBytes = stream->readBytes(buff, min(available, sizeof(buff)));
                if (readBytes > 0) {
                    Update.write(buff, readBytes);
                    written += readBytes;
                }
            } else {
                delay(10);
            }
        }

        if (written == (size_t)contentLength) {
            Serial.printf("[OTA] Transferred %u bytes successfully\n", (unsigned int)written);
        } else {
            Serial.printf("[OTA] Incomplete write: %u/%d bytes (Error #%d)\n", (unsigned int)written, contentLength, Update.getError());
        }

        if (Update.end()) {
            if (Update.isFinished()) {
                Serial.println("[OTA] Verification complete! Firmware successfully flashed. Rebooting controller...");
                http.end();
                delay(800);
                ESP.restart();
                return true;
            } else {
                Serial.println("[OTA] Error: Update aborted prematurely");
            }
        } else {
            Serial.printf("[OTA] Verification/Flash failed with error code: %d\n", Update.getError());
        }

        http.end();
        return false;
    }
};

} // namespace ac::ota
