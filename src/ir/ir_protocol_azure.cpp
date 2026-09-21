#include "ir_protocol_azure.h"

namespace ac::ir {

uint8_t AzureEssenceProtocol::calculateChecksum(const uint8_t state[kAzureStateLength]) {
    uint16_t nibbleSum = 0;
    // Sum all 16 nibbles from byte 0 through byte 7
    for (uint8_t i = 0; i < 8; i++) {
        nibbleSum += (state[i] >> 4) & 0x0F;
        nibbleSum += state[i] & 0x0F;
    }
    return static_cast<uint8_t>(nibbleSum & 0x0F);
}

void AzureEssenceProtocol::encode(const control::ACState& state, uint8_t outBytes[kAzureStateLength]) {
    // Byte 0: Constant Vendor / Header ID
    outBytes[0] = kAzureVendorId;

    // Byte 1: Fan Speed (high nibble) + Operating Mode (low nibble)
    uint8_t fanNibble = kAzureFanAuto;
    if (state.fanSpeed == "med" || state.fanSpeed == "low") {
        fanNibble = kAzureFanMed;
    } else if (state.fanSpeed == "high") {
        fanNibble = kAzureFanHigh;
    }

    uint8_t modeNibble = kAzureModeCool;
    if (state.mode == "fan") {
        modeNibble = kAzureModeFan;
    } else if (state.mode == "dry") {
        modeNibble = kAzureModeDry;
    } else if (state.mode == "auto") {
        modeNibble = kAzureModeAuto;
    }
    outBytes[1] = (fanNibble << 4) | (modeNibble & 0x0F);

    // Bytes 2..5: Reserved / zeros
    outBytes[2] = 0x00;
    outBytes[3] = 0x00;
    outBytes[4] = 0x00;
    outBytes[5] = 0x00;

    // Byte 6: Temperature (clamped 16°C to 31°C)
    // Wire encoding formula: temp_C + 11 (24°C=0x23, 25°C=0x24, 26°C=0x25, 27°C=0x26)
    uint8_t clampedTemp = state.temperature;
    if (clampedTemp < 16) clampedTemp = 16;
    if (clampedTemp > 31) clampedTemp = 31;
    outBytes[6] = clampedTemp + 11;

    // Byte 7: Power state
    outBytes[7] = state.power ? kAzurePowerOnByte : kAzurePowerOffByte;

    // Byte 8: Checksum in high nibble, 0x0 in low nibble
    uint8_t chk = calculateChecksum(outBytes);
    outBytes[8] = (chk << 4) & 0xF0;
}

bool AzureEssenceProtocol::decode(const uint8_t inBytes[kAzureStateLength], control::ACState& outState) {
    // Validate vendor ID
    if (inBytes[0] != kAzureVendorId) {
        return false;
    }

    // Validate checksum
    uint8_t expectedChecksum = calculateChecksum(inBytes);
    uint8_t receivedChecksum = (inBytes[8] >> 4) & 0x0F;
    if (expectedChecksum != receivedChecksum) {
        return false;
    }

    // Power state
    outState.power = (inBytes[7] == kAzurePowerOnByte);

    // Mode
    uint8_t modeNibble = inBytes[1] & 0x0F;
    if (modeNibble == kAzureModeCool) outState.mode = "cool";
    else if (modeNibble == kAzureModeDry) outState.mode = "dry";
    else if (modeNibble == kAzureModeFan) outState.mode = "fan";
    else if (modeNibble == kAzureModeAuto) outState.mode = "auto";
    else outState.mode = "unknown";

    // Fan
    uint8_t fanNibble = (inBytes[1] >> 4) & 0x0F;
    if (fanNibble == kAzureFanAuto) outState.fanSpeed = "auto";
    else if (fanNibble == kAzureFanMed) outState.fanSpeed = "med";
    else if (fanNibble == kAzureFanHigh) outState.fanSpeed = "high";
    else outState.fanSpeed = "auto";

    // Temperature (wire - 11)
    if (inBytes[6] >= 27 && inBytes[6] <= 42) {
        outState.temperature = inBytes[6] - 11;
    } else {
        outState.temperature = 25;
    }

    outState.source = "ir_rx";
    return true;
}

bool AzureEssenceProtocol::decodeRaw(const uint16_t* raw, uint16_t rawLen, control::ACState& outState) {
    if (raw == nullptr || rawLen < 146) {
        return false;
    }

    // Header validation (Mark ~4500us, Space ~2400us with generous sensor tolerance)
    if (raw[0] < 3000 || raw[0] > 6000) return false;
    if (raw[1] < 1500 || raw[1] > 3300) return false;

    uint8_t stateBytes[kAzureStateLength] = {0};
    uint16_t idx = 2;

    for (uint8_t byteIdx = 0; byteIdx < kAzureStateLength; byteIdx++) {
        uint8_t byteVal = 0;
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (idx + 1 >= rawLen) return false;
            uint16_t mark = raw[idx++];
            uint16_t space = raw[idx++];

            if (mark < 150 || mark > 900) return false;

            // Space: logical 1 is ~970us (700-1500us), logical 0 is ~470us (150-699us)
            if (space >= 700 && space <= 1500) {
                byteVal |= (1 << bit);
            } else if (space >= 150 && space < 700) {
                // 0 bit
            } else {
                return false; // Timing violation
            }
        }
        stateBytes[byteIdx] = byteVal;
    }

    return decode(stateBytes, outState);
}

uint16_t AzureEssenceProtocol::generateRaw(const uint8_t state[kAzureStateLength], uint16_t* outRaw, uint16_t maxLen) {
    if (outRaw == nullptr || maxLen < kAzureRawTransitions) {
        return 0;
    }

    uint16_t idx = 0;
    // Header
    outRaw[idx++] = kAzureHdrMark;
    outRaw[idx++] = kAzureHdrSpace;

    // 72 Data bits: LSB first for each byte
    for (uint8_t byteIdx = 0; byteIdx < kAzureStateLength; byteIdx++) {
        uint8_t byteVal = state[byteIdx];
        for (uint8_t bit = 0; bit < 8; bit++) {
            outRaw[idx++] = kAzureBitMark;
            bool isOne = (byteVal >> bit) & 1;
            outRaw[idx++] = isOne ? kAzureOneSpace : kAzureZeroSpace;
        }
    }

    // Trailing stop mark
    outRaw[idx++] = kAzureStopMark;
    return idx;
}

} // namespace ac::ir
