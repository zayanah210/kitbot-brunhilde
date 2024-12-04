#pragma once

#include "Drivetrain.h"
#include "Constants.h"

class Robot : public frc::TimedRobot {
    public:
        Robot();
        ~Robot() {}

        void RobotInit() override;
        void RobotPeriodic() override;
        void DisabledInit() override;
        void DisabledPeriodic() override;
        void AutonomousInit() override;
        void AutonomousPeriodic() override;
        void TeleopPeriodic() override;
        void TestPeriodic() override;
        void AutonomousExit() override;
        
    private:
        valor::Gamepad gamepadDriver{OIConstants::GAMEPAD_BASE_LOCATION};
        valor::Gamepad gamepadOperator{OIConstants::GAMEPAD_OPERATOR_LOCATION};
        Drivetrain drivetrain{this};
};
