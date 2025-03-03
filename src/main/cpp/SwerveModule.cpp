#include "SwerveModule.h"
#include <frc/smartdashboard/SmartDashboard.h>

SwerveModule::SwerveModule(int driveID, int steerID, int cancoderID, double angleOffset)
    : m_driveMotor(driveID, "rio"),
      m_steerMotor(steerID, "rio"),
      m_cancoder(cancoderID, "rio"),
      m_angleOffset(angleOffset) {
  m_driveMotor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
  m_driveMotor.SetInverted(false);
  m_steerMotor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
  m_steerMotor.SetInverted(false);
}

frc::SwerveModuleState SwerveModule::GetState() {
  double velocity = m_driveMotor.GetVelocity().GetValueAsDouble() * kDriveGearRatio * kWheelDiameter * M_PI;
  double angle = m_cancoder.GetAbsolutePosition().GetValueAsDouble() - m_angleOffset;
  return {units::feet_per_second_t(velocity), frc::Rotation2d(units::degree_t(angle))};
}

frc::SwerveModulePosition SwerveModule::GetPosition() {
  double distance = m_driveMotor.GetPosition().GetValueAsDouble() * kDriveGearRatio * kWheelDiameter * M_PI;
  return {units::foot_t(distance), GetState().angle};
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState& state) {
  auto optimizedState = frc::SwerveModuleState::Optimize(state, GetState().angle);

  // Drive: Scale speed to [-1, 1] duty cycle (max speed ~15 ft/s)
  double maxSpeedFps = 15.0; // Match kMaxSpeed in Drivetrain.h
  double driveDutyCycle = optimizedState.speed.value() / maxSpeedFps;
  ctre::phoenix6::controls::DutyCycleOut driveControl{driveDutyCycle};
  m_driveMotor.SetControl(driveControl);

  // Steer: Simple proportional control to target angle
  double currentAngle = GetState().angle.Degrees().value();
  double targetAngle = optimizedState.angle.Degrees().value();
  double error = targetAngle - currentAngle;
  if (error > 180.0) error -= 360.0; // Handle wraparound
  if (error < -180.0) error += 360.0;
  double steerDutyCycle = error * 0.05; // Increased P gain from 0.01 to 0.05
  if (steerDutyCycle > 1.0) steerDutyCycle = 1.0;
  if (steerDutyCycle < -1.0) steerDutyCycle = -1.0;
  ctre::phoenix6::controls::DutyCycleOut steerControl{steerDutyCycle};
  m_steerMotor.SetControl(steerControl);
}