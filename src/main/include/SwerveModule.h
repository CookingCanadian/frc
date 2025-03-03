#pragma once

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h> // Added for GetPosition()
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>

class SwerveModule {
 public:
  SwerveModule(int driveID, int steerID, int cancoderID, double angleOffset);
  frc::SwerveModuleState GetState();
  frc::SwerveModulePosition GetPosition(); // Must be declared here
  void SetDesiredState(const frc::SwerveModuleState& state);

 private:
  ctre::phoenix6::hardware::TalonFX m_driveMotor;
  ctre::phoenix6::hardware::TalonFX m_steerMotor;
  ctre::phoenix6::hardware::CANcoder m_cancoder;
  double m_angleOffset; // Offset to zero the CANcoder

  static constexpr double kWheelDiameter = 2.0 / 12.0; // 2 inches to feet
  static constexpr double kDriveGearRatio = 6.2 / 1.0; // 6.2:1 ratio for our X2 11T setup
};