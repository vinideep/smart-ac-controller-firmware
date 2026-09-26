#pragma once

#include <Arduino.h>
#include "ac_state.h"
#include "../ir/ir_transmitter.h"

namespace ac::control {

/**
 * @brief Abstract interface for Air Conditioner controllers.
 * Decouples high-level climate automation and APIs from specific hardware AC protocols.
 */
class ACController {
public:
    virtual ~ACController() = default;

    virtual void begin() = 0;
    virtual bool setPower(bool power, const String& source = "user") = 0;
    virtual bool setMode(const String& mode, const String& source = "user") = 0;
    virtual bool setTemperature(uint8_t tempC, const String& source = "user") = 0;
    virtual bool setFanSpeed(const String& fanSpeed, const String& source = "user") = 0;
    virtual bool setSwing(bool swing, const String& source = "user") = 0;
    virtual bool setSleep(bool sleep, const String& source = "user") = 0;
    virtual bool setState(bool power, uint8_t tempC, const String& mode, const String& fanSpeed, const String& source = "user") = 0;

    virtual const ACState& getState() const = 0;
    virtual bool sendState(const String& source = "user") = 0;
};

/**
 * @brief Concrete controller for Azure Essence AE-18 split AC unit.
 * Encapsulates validation, capabilities discovered from IR calibration, and state machine transitions.
 */
class AzureEssenceController : public ACController {
public:
    explicit AzureEssenceController(uint8_t txPin);

    void begin() override;

    bool setPower(bool power, const String& source = "user") override;
    bool setMode(const String& mode, const String& source = "user") override;
    bool setTemperature(uint8_t tempC, const String& source = "user") override;
    bool setFanSpeed(const String& fanSpeed, const String& source = "user") override;
    bool setSwing(bool swing, const String& source = "user") override;
    bool setSleep(bool sleep, const String& source = "user") override;
    bool setState(bool power, uint8_t tempC, const String& mode, const String& fanSpeed, const String& source = "user") override;

    const ACState& getState() const override { return _state; }
    bool sendState(const String& source = "user") override;

    void applyExternalState(const ACState& newState);

    // Minimum and maximum validated temperature range
    static constexpr uint8_t kMinTemperature = 16;
    static constexpr uint8_t kMaxTemperature = 31;

    ir::IRTransmitterDriver& getTransmitter() { return _transmitter; }

private:
    ACState _state;
    ir::IRTransmitterDriver _transmitter;
    bool _isDirty = false;

    bool validateTemperature(uint8_t tempC) const;
    bool validateMode(const String& mode) const;
    bool validateFanSpeed(const String& fanSpeed) const;
};

} // namespace ac::control
