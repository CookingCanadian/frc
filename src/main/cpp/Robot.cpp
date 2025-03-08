// Robot.cpp
#include "Robot.h"
#include "RobotContainer.h"

RobotContainer* robotContainer;

void Robot::RobotInit() {
    robotContainer = new RobotContainer();
    m_chooser.AddOption(kAutoNameDefault, kAutoNameDefault);
    m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
    m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
}

void Robot::RobotPeriodic() {
    robotContainer->UpdateOdometry();
}

void Robot::TeleopInit() {
    robotContainer->m_navx->ZeroYaw(); // Reset yaw at teleop start
}

void Robot::TeleopPeriodic() {
    double xSpeed = -robotContainer->m_swerveController.GetRawAxis(OperatorConstants::kForwardAxis);
    double ySpeed = robotContainer->m_swerveController.GetRawAxis(OperatorConstants::kStrafeAxis);
    double rot = robotContainer->m_swerveController.GetRawAxis(OperatorConstants::kRotationAxis);
    bool fieldRelative = robotContainer->m_swerveController.GetRawButton(OperatorConstants::kFieldRelativeButton);

    double triggerValue = robotContainer->m_swerveController.GetLeftTriggerAxis();
    double speedScale = 1.0; // Default to full speed
    if (triggerValue > 0.2) {
        speedScale = 1.0 - ((triggerValue - 0.2) / 0.8) * 0.8;
    }

    xSpeed *= speedScale;
    ySpeed *= speedScale;
    rot *= speedScale;

    robotContainer->Drive(xSpeed, ySpeed, rot, fieldRelative);
}

void Robot::AutonomousInit() {
    m_autoSelected = m_chooser.GetSelected();
}

void Robot::AutonomousPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}
void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif