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
