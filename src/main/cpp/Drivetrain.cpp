#include "Drivetrain.h"
#include "Constants.h"

Drivetrain::Drivetrain(frc::TimedRobot *_robot) :
    BaseSubsystem(_robot, "Drivetrain")
{}

void Drivetrain::init() {
    // TODO: What NEO are we running?
    leftLeader = new valor::NeoController{
        valor::NeoControllerType::NEO_1_1,
        CANIDs::LEFT_MOTORS[0],
        valor::NeutralMode::Brake,
        false,
        CAN_BUS
    };
    rightLeader = new valor::NeoController{
        valor::NeoControllerType::NEO_1_1,
        CANIDs::RIGHT_MOTORS[0],
        valor::NeutralMode::Brake,
        true,
        CAN_BUS
    };
    leftLeader->setupFollower(CANIDs::LEFT_MOTORS[1]);
    rightLeader->setupFollower(CANIDs::RIGHT_MOTORS[1]);

    state.leftSpeed = 0;
    state.rightSpeed = 0;
}

void Drivetrain::assessInputs() {
    if (!driverGamepad || !driverGamepad->IsConnected()) return;

    state.leftSpeed = driverGamepad->leftStickY();
    state.rightSpeed = driverGamepad->rightStickY();
}

void Drivetrain::assignOutputs() {
    leftLeader->setDutyCycle(state.leftSpeed);
    rightLeader->setDutyCycle(state.rightSpeed);
}

void Drivetrain::InitSendable(wpi::SendableBuilder& builder) {
    builder.SetSmartDashboardType("Subsystem");
}