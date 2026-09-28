#pragma once

/**
 * @file pins.h
 * @brief Centralized and configurable GPIO pin definitions for the Smart AC Controller.
 * 
 * All pin assignments are configurable via compile-time preprocessor macros 
 * (-D DHT_PIN=... etc.) or by editing this header.
 * 
 * Hardware Wiring Reference:
 * ----------------------------------------------------
 * Sensor/Module   | Default Pin | Description
 * ----------------------------------------------------
 * DHT22 Data      | GPIO 13*    | Temperature & Humidity Data line
 * Status LED      | GPIO 2      | On-board ESP32 status / blink LED
 * IR Receiver     | GPIO 27     | 38kHz IR receiver demodulator OUT
 * IR Transmitter  | GPIO 25     | Transistor base driving 940nm IR LED
 * IR Obstacle     | GPIO 26     | Proximity / obstacle digital OUT
 * ----------------------------------------------------
 * *Note: Initial project specification proposed GPIO 4 for DHT22.
 * The currently wired physical testbed uses GPIO 13.
 * Both are fully supported by setting DHT_PIN accordingly.
 */

#ifndef DHT_PIN
#define DHT_PIN 13
#endif

#ifndef DHT_TYPE
#define DHT_TYPE DHT22
#endif

#ifndef STATUS_LED_PIN
#define STATUS_LED_PIN 2
#endif

#ifndef IR_RX_PIN
#define IR_RX_PIN 14
#endif

#ifndef IR_TX_PIN
#define IR_TX_PIN 25
#endif

#ifndef IR_OBSTACLE_PIN
#define IR_OBSTACLE_PIN 26
#endif

#ifndef PRESENCE_PIN
#define PRESENCE_PIN 26
#endif

#ifndef PRESENCE_ACTIVE_LOW
#define PRESENCE_ACTIVE_LOW false // Default Active HIGH for radar/mmWave sensors (e.g. RCWL-0516, HLK-LD2410)
#endif

#ifndef RADAR_RX_PIN
#define RADAR_RX_PIN 16 // ESP32 RX2 (connect to Radar TX)
#endif

#ifndef RADAR_TX_PIN
#define RADAR_TX_PIN 17 // ESP32 TX2 (connect to Radar RX)
#endif

#ifndef RADAR_BAUD_RATE
#define RADAR_BAUD_RATE 115200
#endif

#ifndef CURRENT_SENSOR_PIN
#define CURRENT_SENSOR_PIN 34 // ADC1_CH6 (Analog input for SCT-013 or ACS712 CT current sensor)
#endif

#ifndef HAS_CURRENT_SENSOR
#define HAS_CURRENT_SENSOR false // Set true if physical CT clamp / current sensor is wired to CURRENT_SENSOR_PIN
#endif

