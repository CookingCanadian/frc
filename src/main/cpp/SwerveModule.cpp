// SwerveModule.cpp
#include "RobotContainer.h"
#include <frc/controller/PIDController.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <ctre/phoenix6/controls/DutyCycleOut.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

SwerveModule::SwerveModule(int driveID, int steerID, int encoderID, double offset)
    : m_driveMotor(driveID),
      m_steerMotor(steerID),
      m_encoder(encoderID),
      m_angleOffset(offset), // Radians from constructor
      m_steerPID(0.005, 0.03, 0.0002) { // Your working PID
    
    ctre::phoenix6::configs::TalonFXConfiguration config{};
    config.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
    m_driveMotor.GetConfigurator().Apply(config);
    m_steerMotor.GetConfigurator().Apply(config);

    m_steerPID.EnableContinuousInput(-M_PI, M_PI); // Radians
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState& state) {
    auto encoderTurns = m_encoder.GetAbsolutePosition().GetValue();
    auto currentAngle = units::radian_t{encoderTurns.value() * 2.0 * M_PI};
    auto adjustedAngle = currentAngle - units::radian_t{m_angleOffset};
    
    auto optimizedState = frc::SwerveModuleState::Optimize(state, frc::Rotation2d(adjustedAngle));
    
    double driveOutput = optimizedState.speed / AutoConstants::kMaxSpeed;
    units::radian_t steerError = optimizedState.angle.Radians() - adjustedAngle;
    double steerOutput = m_steerPID.Calculate(steerError.value(), 0.0);

    m_driveMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut{driveOutput});
    m_steerMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut{steerOutput});
}

frc::SwerveModulePosition SwerveModule::GetPosition() {
    auto drivePosition = m_driveMotor.GetPosition().GetValue();
    constexpr double kWheelDiameter = 0.1016;
    constexpr double kGearRatio = 6.75;
    units::meter_t distance = units::meter_t{drivePosition.value() * (kWheelDiameter * M_PI) / kGearRatio};

    auto encoderTurns = m_encoder.GetAbsolutePosition().GetValue();
    auto currentAngle = units::radian_t{encoderTurns.value() * 2.0 * M_PI};
    auto adjustedAngle = currentAngle - units::radian_t{m_angleOffset};

    return frc::SwerveModulePosition{distance, frc::Rotation2d(adjustedAngle)};
}

frc::SwerveModuleState SwerveModule::GetState() {
    auto driveVelocity = m_driveMotor.GetVelocity().GetValue();
    constexpr double kWheelDiameter = 0.1016;
    constexpr double kGearRatio = 6.75;
    units::meters_per_second_t speed = units::meters_per_second_t{driveVelocity.value() * (kWheelDiameter * M_PI) / kGearRatio};

    auto encoderTurns = m_encoder.GetAbsolutePosition().GetValue();
    auto currentAngle = units::radian_t{encoderTurns.value() * 2.0 * M_PI};
    auto adjustedAngle = currentAngle - units::radian_t{m_angleOffset};

    return frc::SwerveModuleState{speed, frc::Rotation2d(adjustedAngle)};
}