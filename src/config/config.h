#pragma once
#include <Arduino.h>

/**
 * @file config.h
 * @brief System-wide configuration constants for Azure Essence Smart AC Controller.
 */

#ifndef FIRMWARE_VERSION
#define FIRMWARE_VERSION "0.1.0-alpha"
#endif

#ifndef DEVICE_MODEL
#define DEVICE_MODEL "AzureEssence-SmartAC"
#endif

#ifndef MONITOR_BAUD_RATE
#define MONITOR_BAUD_RATE 115200
#endif

// DHT22 Sampling Configuration
constexpr uint32_t DHT_SAMPLE_INTERVAL_MS = 2000; // Sample every 2 seconds
constexpr uint32_t DHT_READ_TIMEOUT_MS = 250;     // Sensor read timeout

// DHT22 Physical Operational Boundaries
constexpr float DHT_TEMP_MIN_C = -40.0f;
constexpr float DHT_TEMP_MAX_C = 80.0f;
constexpr float DHT_HUMIDITY_MIN = 0.0f;
constexpr float DHT_HUMIDITY_MAX = 100.0f;
constexpr uint8_t DHT_MAX_CONSECUTIVE_ERRORS = 5;

// Watchdog Timer (in seconds)
constexpr uint32_t WDT_TIMEOUT_SECONDS = 8;

// Wi-Fi Configuration
#ifndef WIFI_SSID
#define WIFI_SSID "Deeps" // Set your home Wi-Fi SSID; empty string triggers standalone AP mode
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "9600270332" // Set your home Wi-Fi password
#endif

#ifndef WIFI_HOSTNAME
#define WIFI_HOSTNAME "smart-ac"
#endif

// ──────────────────────────────────────────────────────────────────
// Cloud Backend Configuration
// Set BACKEND_URL to your Railway backend URL after deployment.
// Set DEVICE_TOKEN to any random secret string (same value goes in Railway env vars).
// Leave BACKEND_URL empty to disable cloud push (local-only / serial mode).
// ──────────────────────────────────────────────────────────────────
#ifndef BACKEND_URL
#define BACKEND_URL "https://deepi-home-server.tail07616e.ts.net"  // e.g. "https://smart-ac.up.railway.app"
#endif

#ifndef DEVICE_TOKEN
#define DEVICE_TOKEN "change-me-to-a-secret-token"
#endif

// How often ESP32 pushes telemetry to backend (milliseconds)
constexpr uint32_t CLOUD_TELEMETRY_INTERVAL_MS = 3000;

// How often ESP32 polls backend for pending commands (milliseconds)
constexpr uint32_t CLOUD_POLL_INTERVAL_MS = 2000;

