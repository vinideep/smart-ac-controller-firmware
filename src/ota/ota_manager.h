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
    static String getLastError() {
        return getLastErrorStorage();
    }

    static void setLastError(const String& err) {
        getLastErrorStorage() = err;
    }

    static bool performOta(const String& url, const String& expectedMd5 = "") {
        setLastError("");
        if (WiFi.status() != WL_CONNECTED) {
            setLastError("Wi-Fi not connected");
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
            setLastError(String("HTTP GET failed with status: ") + httpCode);
            Serial.printf("[OTA] HTTP GET failed, error code: %d\n", httpCode);
            http.end();
            return false;
        }

        int contentLength = http.getSize();
        if (contentLength <= 0) {
            setLastError("Invalid Content-Length or chunked encoding unsupported");
            Serial.println("[OTA] Error: Content-Length invalid or chunked encoding unsupported");
            http.end();
            return false;
        }

        if (expectedMd5.length() > 0) {
            Update.setMD5(expectedMd5.c_str());
        }

        bool canBegin = Update.begin(contentLength);
        if (!canBegin) {
            setLastError(String("Insufficient flash partition space for update (") + contentLength + " bytes needed)");
            Serial.printf("[OTA] Error: Insufficient flash partition space for OTA update (%d bytes needed)\n", contentLength);
            http.end();
            return false;
        }

        WiFiClient* stream = http.getStreamPtr();
        size_t written = 0;
        uint8_t buff[1024];
        unsigned long lastReadMs = millis();

        while (http.connected() && (written < (size_t)contentLength)) {
            esp_task_wdt_reset();
            size_t available = stream->available();
            if (available) {
                int readBytes = stream->readBytes(buff, min(available, sizeof(buff)));
                if (readBytes > 0) {
                    if (written == 0 && buff[0] != 0xE9) {
                        setLastError(String("Invalid firmware image magic byte: 0x") + String(buff[0], HEX) + " (expected 0xE9)");
                        Serial.printf("[OTA] Error: Invalid firmware magic byte: 0x%02X (expected 0xE9)\n", buff[0]);
                        Update.abort();
                        http.end();
                        return false;
                    }
                    size_t w = Update.write(buff, readBytes);
                    if (w != (size_t)readBytes || Update.hasError()) {
                        setLastError(String("Flash write failed at byte ") + (unsigned int)written + " (Error #" + Update.getError() + ")");
                        Serial.printf("[OTA] Error: Flash write failed at byte %u (Error #%d)\n", (unsigned int)written, Update.getError());
                        Update.abort();
                        http.end();
                        return false;
                    }
                    written += readBytes;
                    lastReadMs = millis();
                }
            } else {
                if (millis() - lastReadMs > 15000) {
                    setLastError("OTA download stream timed out (no data for 15s)");
                    Serial.println("[OTA] Error: Stream read timeout");
                    Update.abort();
                    http.end();
                    return false;
                }
                delay(10);
            }
        }

        if (written != (size_t)contentLength) {
            setLastError(String("Incomplete write: ") + (unsigned int)written + "/" + contentLength + " bytes (Error #" + Update.getError() + ")");
            Serial.printf("[OTA] Incomplete write: %u/%d bytes (Error #%d)\n", (unsigned int)written, contentLength, Update.getError());
            Update.abort();
            http.end();
            return false;
        }

        Serial.printf("[OTA] Transferred %u bytes successfully\n", (unsigned int)written);

        if (Update.end()) {
            if (Update.isFinished()) {
                Serial.println("[OTA] Verification complete! Firmware successfully flashed. Rebooting controller...");
                http.end();
                delay(800);
                ESP.restart();
                return true;
            } else {
                setLastError("Update aborted prematurely");
                Serial.println("[OTA] Error: Update aborted prematurely");
            }
        } else {
            setLastError(String("Verification/Flash failed with error code: ") + Update.getError());
            Serial.printf("[OTA] Verification/Flash failed with error code: %d\n", Update.getError());
        }

        http.end();
        return false;
    }

private:
    static String& getLastErrorStorage() {
        static String s_lastError;
        return s_lastError;
    }
};

} // namespace ac::ota
