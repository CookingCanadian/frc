#include "Drivetrain.h"
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/smartdashboard/SmartDashboard.h>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override {}
  void RobotPeriodic() override {
    m_drivetrain.UpdateOdometry();
    frc::SmartDashboard::PutNumber("X", m_drivetrain.GetPose().X().value());
    frc::SmartDashboard::PutNumber("Y", m_drivetrain.GetPose().Y().value());
    frc::SmartDashboard::PutNumber("Heading", m_drivetrain.GetPose().Rotation().Degrees().value());
  }

  void TeleopInit() override {}
  void TeleopPeriodic() override {
    double xSpeed = -m_controller.GetLeftY() * 15.0; // Max 15 ft/s (forward/back)
    double ySpeed = -m_controller.GetLeftX() * 15.0; // Max 15 ft/s (strafe left/right)
    double rotSpeed = -m_controller.GetRightX() * 2 * M_PI; // Max 1 rotation/s

    // Smaller deadband for better responsiveness
    if (std::abs(xSpeed) < 0.05) xSpeed = 0.0;
    if (std::abs(ySpeed) < 0.05) ySpeed = 0.0;
    if (std::abs(rotSpeed) < 0.05) rotSpeed = 0.0;

    // Debug joystick inputs
    frc::SmartDashboard::PutNumber("xSpeed", xSpeed);
    frc::SmartDashboard::PutNumber("ySpeed", ySpeed);
    frc::SmartDashboard::PutNumber("rotSpeed", rotSpeed);

    m_drivetrain.Drive(
        units::feet_per_second_t(xSpeed),
        units::feet_per_second_t(ySpeed),
        units::radians_per_second_t(rotSpeed)
    );
  }

 private:
  Drivetrain m_drivetrain;
  frc::XboxController m_controller{0};
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif