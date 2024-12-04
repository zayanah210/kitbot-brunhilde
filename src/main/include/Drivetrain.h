#pragma once

#include "valkyrie/BaseSubsystem.h"
#include "valkyrie/controllers/NeoController.h"

class Drivetrain : public valor::BaseSubsystem {
public:
    Drivetrain(frc::TimedRobot*);
    void init() override;
    void assessInputs() override;
    void assignOutputs() override;
    void InitSendable(wpi::SendableBuilder&);

private:

    struct {
        double leftSpeed;
        double rightSpeed;
    } state;

    valor::NeoController *leftLeader;
    valor::NeoController *rightLeader;
};