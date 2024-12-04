#include "valkyrie/controllers/NeoController.h"

#define NEO_PIDF_KP 10.0f
#define NEO_PIDF_KI 0.0f
#define NEO_PIDF_KD 0.0f
#define NEO_PIDF_KF 0.0f

const units::turns_per_second_t NEO_PIDF_KV(6); // RPS cruise velocity
const units::turns_per_second_squared_t NEO_PIDF_KA(130.0); // RPS/S acceleration (6.5/130 = 0.05 seconds to max speed)
const units::turns_per_second_cubed_t NEO_PIDF_KJ(650.0); // RPS/S^2 jerk (4000/40000 = 0.1 seconds to max acceleration)

const units::ampere_t SUPPLY_CURRENT_THRESHOLD(60);
const units::ampere_t STATOR_CURRENT_LIMIT(80);
const units::ampere_t SUPPLY_CURRENT_LIMIT(45);
const units::millisecond_t SUPPLY_TIME_THRESHOLD(500);

using namespace valor;


NeoController::NeoController(valor::NeoControllerType controllerType,
                                    int canID,
                                    valor::NeutralMode _mode,
                                    bool _inverted,
                                    std::string canbus) :
    BaseController(new rev::spark::SparkMax(canID, rev::spark::SparkMax::MotorType::kBrushless), _inverted, _mode, getMotorFreeSpeed(controllerType))
{
    init();
}

void NeoController::init()
{
    valor::PIDF motionPIDF;
    motionPIDF.P = NEO_PIDF_KP;
    motionPIDF.I = NEO_PIDF_KI;
    motionPIDF.D = NEO_PIDF_KD;

    config.VoltageCompensation(getVoltageCompensation().to<double>());
    setNeutralMode(neutralMode);
    // set smart current limit
    // set hard current limit
    setGearRatios(rotorToSensor, sensorToMech);
    applyConfig();

    wpi::SendableRegistry::AddLW(this, "NeoController", "ID " + std::to_string(0));
}

void NeoController::setupCANCoder(int deviceId, units::turn_t offset, bool clockwise, std::string canbus, ctre::phoenix6::signals::AbsoluteSensorRangeValue absoluteRange, bool saveImmediately)
{
    // It's way too much work to setup a cancoder with Spark Max just to get very little compatibility between them
    // Spark Max doesn't support using any external encoders for their internal PID loop
}

void NeoController::applyConfig()
{
    // FIXME: Handle error
    motor->Configure(
        config,
        rev::spark::SparkBase::ResetMode::kResetSafeParameters,
        rev::spark::SparkBase::PersistMode::kPersistParameters
    );
}

units::turn_t NeoController::getCANCoder()
{
    return 0_tr;
}

void NeoController::reset()
{
    setEncoderPosition(0_tr);
}

void NeoController::setEncoderPosition(units::turn_t position)
{
    // FIXME: units
    config.encoder.PositionConversionFactor(1);
    motor->GetEncoder().SetPosition(position.to<double>());
}

void NeoController::setupFollower(int canID, bool followerInverted)
{
    followerMotor = new rev::spark::SparkMax{canID, rev::spark::SparkLowLevel::MotorType::kBrushless};
    // Uses the same leader config but applies it only on FOLLOWER
    config.Follow(*motor, followerInverted);
    followerMotor->Configure(config, rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);
    config.DisableFollowerMode();
}

void NeoController::setForwardLimit(units::turn_t forward, bool saveImmediately)
{
    // FIXME: units
    config.softLimit.ForwardSoftLimitEnabled(true).ForwardSoftLimit(forward.to<double>());
    if (saveImmediately) applyConfig();
}

void NeoController::setReverseLimit(units::turn_t reverse, bool saveImmediately)
{
    config.softLimit.ReverseSoftLimitEnabled(true).ReverseSoftLimit(reverse.to<double>());
    if (saveImmediately) applyConfig();
}

void NeoController::setSmartCurrentLimit(units::ampere_t stallLimit, units::ampere_t freeLimit, units::turns_per_second_t limitTPS, bool saveImmediately)
{
    config.SmartCurrentLimit(
        stallLimit.to<double>(),
        freeLimit.to<double>(),
        limitTPS.to<double>() / 60
    );
    if (saveImmediately) applyConfig();
}

void NeoController::setHardCurrentLimit(units::ampere_t limit, int chopCycles, bool saveImmediately) {
    config.SecondaryCurrentLimit(limit.to<double>(), chopCycles);
    if (saveImmediately) applyConfig();
}

void NeoController::setPIDF(valor::PIDF pidf, int slot, bool saveImmediately)
{
    config.closedLoop.Pidf(pidf.P, pidf.I, pidf.D, pidf.aFF);
    config.closedLoop.maxMotion.AllowedClosedLoopError(pidf.error.to<double>());
    config.closedLoop.maxMotion.MaxVelocity(pidf.maxVelocity.to<double>());
    config.closedLoop.maxMotion.MaxAcceleration(pidf.maxAcceleration.to<double>());
    if (saveImmediately) applyConfig();
}

void NeoController::setGearRatios(double _rotorToSensor, double _sensorToMech, bool saveImmediately)
{
    rotorToSensor = _rotorToSensor;
    sensorToMech = _sensorToMech;

    // Converts 
    config.encoder.VelocityConversionFactor(60 * sensorToMech);
    config.encoder.PositionConversionFactor(60 * sensorToMech);
    if (saveImmediately) applyConfig();
}

units::ampere_t NeoController::getCurrent()
{
    return units::ampere_t{motor->GetOutputCurrent()};
}

/**
 * Output is in mechanism rotations!
*/
units::turn_t NeoController::getPosition()
{
    return units::turn_t{motor->GetEncoder().GetPosition()};
}

/**
 * Output is in mechanism rotations!
*/
units::turns_per_second_t NeoController::getSpeed()
{
    return units::turns_per_second_t{motor->GetEncoder().GetVelocity()};
}

/**
 * Set a position in mechanism rotations
*/
void NeoController::setPosition(units::turn_t position)
{
    motor->GetClosedLoopController().SetReference(
        position.to<double>(),
        rev::spark::SparkLowLevel::ControlType::kMAXMotionPositionControl
    );
}

void NeoController::setSpeed(units::turns_per_second_t speed)
{
    // TPS -> RPM
    motor->GetClosedLoopController().SetReference(
        speed.to<double>(),
        rev::spark::SparkLowLevel::ControlType::kMAXMotionVelocityControl
    );
}

void NeoController::setPower(units::volt_t voltage)
{
    motor->GetClosedLoopController().SetReference(
        voltage.to<double>(),
        rev::spark::SparkLowLevel::ControlType::kVoltage
    );
}

void NeoController::setDutyCycle(units::scalar_t d) {
    motor->GetClosedLoopController().SetReference(
        d.to<double>(),
        rev::spark::SparkLowLevel::ControlType::kDutyCycle
    );
}

void NeoController::setProfile(int profile)
{
    currentProfile = profile;
}

void NeoController::setNeutralMode(valor::NeutralMode _mode, bool saveImmediately)
{
    config.SetIdleMode(
        _mode == valor::NeutralMode::Brake ?
        rev::spark::SparkBaseConfig::IdleMode::kBrake :
        rev::spark::SparkBaseConfig::IdleMode::kCoast
    );

    if (saveImmediately) applyConfig();
}

void NeoController::setOpenLoopRamp(units::second_t time, bool saveImmediately)
{
    config.OpenLoopRampRate(time.to<double>());

    if (saveImmediately) applyConfig();
}

void NeoController::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("Subsystem");
    // builder.AddDoubleProperty(
    //     "Stator Current", 
    //     [this] { return getMotor()->GetStatorCurrent().GetValueAsDouble(); },
    //     nullptr);
    // builder.AddDoubleProperty(
    //     "Supply Current", 
    //     [this] { return getMotor()->GetSupplyCurrent().GetValueAsDouble(); },
    //     nullptr);
    // builder.AddDoubleProperty(
    //     "Device Temp", 
    //     [this] { return getMotor()->GetDeviceTemp().GetValueAsDouble(); },
    //     nullptr);
    // builder.AddDoubleProperty(
    //     "Processor Temp", 
    //     [this] { return getMotor()->GetProcessorTemp().GetValueAsDouble(); },
    //     nullptr);
    // builder.AddDoubleProperty(
    //     "Position", 
    //     [this] { return getPosition().to<double>(); },
    //     nullptr);
    // builder.AddDoubleProperty(
    //     "Speed", 
    //     [this] { return getSpeed().to<double>(); },
    //     nullptr);
    // builder.AddDoubleProperty(
    //     "Out Volt", 
    //     [this] { return getMotor()->GetMotorVoltage().GetValueAsDouble(); },
    //     nullptr);
    // builder.AddDoubleProperty(
    //     "CANCoder", 
    //     [this] { return getCANCoder().to<double>(); },
    //     nullptr);
    // builder.AddBooleanProperty(
    //     "Undervolting",
    //     [this] { return getMotor()->GetFault_Undervoltage().GetValue(); },
    //     nullptr);
    // builder.AddIntegerProperty(
    //     "Device ID",
    //     [this] { return getMotor()->GetDeviceID(); },
    //     nullptr
    //     );
    // builder.AddIntegerProperty(
    //     "RotorToSensor",
    //     [this] { return rotorToSensor; },
    //     nullptr);
    // builder.AddIntegerProperty(
    //     "SensorToMech",
    //     [this] { return sensorToMech; },
    //     nullptr
    //     );
}
