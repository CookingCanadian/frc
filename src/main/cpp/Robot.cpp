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
    robotContainer->m_navx->ZeroYaw();
}

void Robot::TeleopPeriodic() {
    double xSpeed = -robotContainer->m_swerveController.GetRawAxis(OperatorConstants::kForwardAxis);
    double ySpeed = robotContainer->m_swerveController.GetRawAxis(OperatorConstants::kStrafeAxis);
    double rot = robotContainer->m_swerveController.GetRawAxis(OperatorConstants::kRotationAxis);
    bool fieldRelative = robotContainer->m_swerveController.GetRawButton(OperatorConstants::kFieldRelativeButton);

    double triggerValue = robotContainer->m_swerveController.GetLeftTriggerAxis();
    double speedScale = 1.0;
    if (triggerValue > 0.2) {
        speedScale = 1.0 - ((triggerValue - 0.2) / 0.8) * 0.8;
    }

    xSpeed *= speedScale;
    ySpeed *= -speedScale;
    rot *= speedScale;

    robotContainer->Drive(xSpeed, ySpeed, rot, fieldRelative);

    double mechanismY = -robotContainer->m_elevatorController.GetRawAxis(frc::XboxController::Axis::kLeftY);
    robotContainer->SetMechanismPosition(mechanismY);
}

void Robot::AutonomousInit() {
    autoStartTime = frc::Timer::GetFPGATimestamp();
    m_autoSelected = m_chooser.GetSelected();
}

void Robot::AutonomousPeriodic() {
    auto now = frc::Timer::GetFPGATimestamp();
    double elapsed = (now - autoStartTime).value();

    if (elapsed < 2.0) {
        robotContainer->Drive(0.2, 0, 0, true);
        auto pose = robotContainer->GetPose(); 
        if (pose.Y() > 1_m) {
            robotContainer->Drive(0, 0, 0, true);
        }
    } else {
        robotContainer->Drive(0, 0, 0, true);
    }
}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}
void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif