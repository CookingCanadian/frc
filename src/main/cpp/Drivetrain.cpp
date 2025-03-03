#include "Drivetrain.h"
#include <frc/smartdashboard/SmartDashboard.h>

// Helper function for initial positions
std::array<frc::SwerveModulePosition, 4> MakeInitialPositions() {
  return std::array<frc::SwerveModulePosition, 4>{
      frc::SwerveModulePosition{0_m, frc::Rotation2d()},
      frc::SwerveModulePosition{0_m, frc::Rotation2d()},
      frc::SwerveModulePosition{0_m, frc::Rotation2d()},
      frc::SwerveModulePosition{0_m, frc::Rotation2d()}
  };
}

Drivetrain::Drivetrain()
    : m_initialPositions(MakeInitialPositions()),
      m_odometry(m_kinematics, frc::Rotation2d(), m_initialPositions) {
}

void Drivetrain::Drive(units::feet_per_second_t xSpeed, units::feet_per_second_t ySpeed, units::radians_per_second_t rotSpeed) {
  frc::ChassisSpeeds speeds{xSpeed, ySpeed, rotSpeed};
  auto states = m_kinematics.ToSwerveModuleStates(speeds);
  m_kinematics.DesaturateWheelSpeeds(&states, kMaxSpeed);

  m_frontLeft.SetDesiredState(states[0]);
  m_frontRight.SetDesiredState(states[1]);
  m_backLeft.SetDesiredState(states[2]);
  m_backRight.SetDesiredState(states[3]);
}

frc::Pose2d Drivetrain::GetPose() const {
  return m_odometry.GetPose();
}

void Drivetrain::UpdateOdometry() {
  // Use actual module positions from SwerveModule::GetPosition()
  std::array<frc::SwerveModulePosition, 4> positions = {
      m_frontLeft.GetPosition(),
      m_frontRight.GetPosition(),
      m_backLeft.GetPosition(),
      m_backRight.GetPosition()
  };

  m_odometry.Update(
      frc::Rotation2d(units::degree_t(m_dummyHeading)),
      positions // Pass array of positions
  );
}