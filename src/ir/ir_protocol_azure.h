#pragma once
#include <stdint.h>
#include <stddef.h>
#include "../ac/ac_state.h"

namespace ac::ir {

constexpr uint16_t kAzureCarrierFreq = 38000;    // 38 kHz carrier
constexpr uint16_t kAzureHdrMark     = 4500;     // Header mark in us
constexpr uint16_t kAzureHdrSpace    = 2400;     // Header space in us
constexpr uint16_t kAzureBitMark     = 430;      // Bit mark in us
constexpr uint16_t kAzureOneSpace    = 970;      // Logical 1 space in us
constexpr uint16_t kAzureZeroSpace   = 470;      // Logical 0 space in us
constexpr uint16_t kAzureStopMark    = 430;      // Trailing stop mark in us
constexpr uint16_t kAzureRepeatSpace = 21000;    // Inter-frame repeat gap in us

constexpr uint8_t  kAzureStateLength = 9;        // 9 bytes (72 bits)
constexpr uint16_t kAzureRawTransitions = 147;   // 2 (hdr) + 72*2 (data) + 1 (stop)

// Constant protocol bytes
constexpr uint8_t kAzureVendorId     = 0x19;
constexpr uint8_t kAzurePowerOnByte  = 0x18;
constexpr uint8_t kAzurePowerOffByte = 0x70;

// Mode constants
constexpr uint8_t kAzureModeCool     = 0x02;
constexpr uint8_t kAzureModeAuto     = 0x01;
constexpr uint8_t kAzureModeDry      = 0x03;
constexpr uint8_t kAzureModeFan      = 0x04;

// Fan constants
constexpr uint8_t kAzureFanAuto      = 0x01;
constexpr uint8_t kAzureFanMed       = 0x02;
constexpr uint8_t kAzureFanHigh      = 0x03;

class AzureEssenceProtocol {
public:
    /**
     * @brief Compute the 4-bit nibble checksum across bytes 0..7.
     * Formula: sum of all 16 nibbles modulo 16.
     */
    static uint8_t calculateChecksum(const uint8_t state[kAzureStateLength]);

    /**
     * @brief Encode full AC state into the 9-byte Azure Essence wire format.
     */
    static void encode(const control::ACState& state, uint8_t outBytes[kAzureStateLength]);

    /**
     * @brief Decode 9-byte state back into an ACState struct.
     */
    static bool decode(const uint8_t inBytes[kAzureStateLength], control::ACState& outState);

    /**
     * @brief Generate raw microsecond timing transitions from 9-byte state for IRsend::sendRaw.
     * @param state 9-byte wire representation
     * @param outRaw Output array (must have space for at least kAzureRawTransitions elements)
     * @param maxLen Size of outRaw array
     * @return Number of transitions generated (147)
     */
    static uint16_t generateRaw(const uint8_t state[kAzureStateLength], uint16_t* outRaw, uint16_t maxLen);
};

} // namespace ac::ir
