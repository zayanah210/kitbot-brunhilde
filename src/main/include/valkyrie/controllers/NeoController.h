#pragma once

#include "valkyrie/controllers/BaseController.h"
#include <string>
#include <rev/SparkMax.h>
#include <rev/AbsoluteEncoder.h>
#include <rev/RelativeEncoder.h>
#include <rev/config/ClosedLoopConfig.h>

namespace valor {

enum NeoControllerType {
    NEO_1_1, // Neo V1.1
    NEO_550
};

constexpr auto NEO_1_1_FREE_SPD = 5676_rpm;
constexpr auto NEO_550_FREE_SPD = 11000_rpm;

class NeoController : public BaseController<rev::spark::SparkMax>
{
public:
    NeoController(valor::NeoControllerType, int _canID, valor::NeutralMode _mode, bool _inverted, std::string _canbus = "");

    void init() override;

    void applyConfig() override;

    void reset() override;
    void setNeutralMode(valor::NeutralMode mode, bool saveImmediately = false) override;
    
    void setSmartCurrentLimit(units::ampere_t stallLimit, units::ampere_t freeLimit, units::turns_per_second_t limitTPS = 20000_tps, bool saveImmediately = false);
    void setHardCurrentLimit(units::ampere_t limit, int chopCycles = 0, bool saveImmediately = false);

    units::turn_t getPosition() override;
    units::turns_per_second_t getSpeed() override;
    units::ampere_t getCurrent() override;

    void setEncoderPosition(units::turn_t position) override;
    
    void setPosition(units::turn_t) override;
    void setSpeed(units::turns_per_second_t) override;
    void setPower(units::volt_t) override;
    void setDutyCycle(units::scalar_t) override;

    void setupFollower(int, bool = false) override;
    
    void setPIDF(valor::PIDF pidf, int slot, bool saveImmediately = false) override;

    void setForwardLimit(units::turn_t forward, bool saveImmediately = false) override;
    void setReverseLimit(units::turn_t reverse, bool saveImmediately = false) override;
    
    void setGearRatios(double, double, bool saveImmediately = false) override;

    void setProfile(int slot) override;

    void setupCANCoder(int deviceId, units::turn_t offset, bool clockwise, std::string canbus = "", ctre::phoenix6::signals::AbsoluteSensorRangeValue absoluteRange=ctre::phoenix6::signals::AbsoluteSensorRangeValue::Unsigned_0To1, bool saveImmediately = false) override;
    units::turn_t getCANCoder() override;
    
    void setOpenLoopRamp(units::second_t time, bool saveImmediately = false) override;

    units::turn_t getAbsEncoderPosition() { return 0_tr; }

    void InitSendable(wpi::SendableBuilder& builder) override;

private:
    constexpr static units::revolutions_per_minute_t getMotorFreeSpeed(NeoControllerType t) {
        switch (t) {
        case NEO_1_1: return NEO_1_1_FREE_SPD;
        case NEO_550: return NEO_550_FREE_SPD;
        default: throw std::logic_error{"Invalid NeoControllerType"};
        };
    }

    int currentProfile;
    rev::spark::SparkBaseConfig config;
};
}
