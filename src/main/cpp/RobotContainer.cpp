// RobotContainer.cpp
#include "RobotContainer.h"
#include <frc/geometry/Rotation2d.h>
#include <thread>
#include <chrono>

RobotContainer::RobotContainer()
    : m_navx(new studica::AHRS(studica::AHRS::NavXComType::kMXP_SPI, 200)),
      m_odometry(m_kinematics, 
                 frc::Rotation2d{0_rad}, 
                 {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                  m_backLeft.GetPosition(), m_backRight.GetPosition()},
                 frc::Pose2d{}) {
    // Wait for NavX to connect
    int attempts = 0;
    const int maxAttempts = 250; // ~5 seconds at 20ms per attempt
    while (!m_navx->IsConnected() && attempts < maxAttempts) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        attempts++;
    }

    // If connected, wait for calibration and zero yaw
    if (m_navx->IsConnected()) {
        while (m_navx->IsCalibrating()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        m_navx->ZeroYaw();
    }

    // Configure leader motor (m_elevatorPivot, CAN ID 13)
    m_elevatorPivot.SetInverted(false);  // Set inversion directly
    rev::spark::SparkBaseConfig leaderConfig{};
    leaderConfig
        .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake)  // Brake mode
        .closedLoop
            .SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)  // NEO encoder
            .Pid(0.1, 0.0, 0.0)  // P=0.1, tune as needed
        .OutputRange(-0.5, 0.5);
    m_elevatorPivot.Configure(leaderConfig, rev::spark::SparkBase::ResetMode::kResetSafeParameters,
                              rev::spark::SparkBase::PersistMode::kPersistParameters);

    // Configure follower motor (m_elevatorPivot2, CAN ID 14)
    m_elevatorPivot2.SetInverted(true);  // Set inversion directly
    rev::spark::SparkBaseConfig followerConfig{};
    followerConfig
        .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake)
        .Follow(m_elevatorPivot.GetDeviceId(), true);
    m_elevatorPivot2.Configure(followerConfig, rev::spark::SparkBase::ResetMode::kResetSafeParameters,
                               rev::spark::SparkBase::PersistMode::kPersistParameters);
}

RobotContainer::~RobotContainer() {
    delete m_navx;
}

void RobotContainer::Drive(double xSpeed, double ySpeed, double rot, bool fieldRelative) {
    // Apply deadband from OperatorConstants
    if (std::abs(xSpeed) < OperatorConstants::kDeadband) xSpeed = 0.0;
    if (std::abs(ySpeed) < OperatorConstants::kDeadband) ySpeed = 0.0;
    if (std::abs(rot) < OperatorConstants::kDeadband) rot = 0.0;

    // Convert to chassis speeds
    frc::ChassisSpeeds speeds;
    if (fieldRelative) {
        speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
            xSpeed * AutoConstants::kMaxSpeed,
            ySpeed * AutoConstants::kMaxSpeed,
            rot * AutoConstants::kMaxAngularSpeed,
            frc::Rotation2d{units::degree_t{m_navx->GetYaw()}});
    } else {
        speeds = frc::ChassisSpeeds{
            xSpeed * AutoConstants::kMaxSpeed,
            ySpeed * AutoConstants::kMaxSpeed,
            rot * AutoConstants::kMaxAngularSpeed};
    }

    // Convert to module states and desaturate
    auto states = m_kinematics.ToSwerveModuleStates(speeds);
    m_kinematics.DesaturateWheelSpeeds(&states, AutoConstants::kMaxSpeed);

    // Apply to modules
    m_frontLeft.SetDesiredState(states[0]);
    m_frontRight.SetDesiredState(states[1]);
    m_backLeft.SetDesiredState(states[2]);
    m_backRight.SetDesiredState(states[3]);
}

void RobotContainer::UpdateOdometry() {
    m_odometry.Update(
        frc::Rotation2d{units::degree_t{m_navx->GetYaw()}},
        {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
         m_backLeft.GetPosition(), m_backRight.GetPosition()}
    );
}

void RobotContainer::SetMechanismPosition(double joystickY) {
    constexpr double kMaxPosition = 10.0;  // Max rotations
    constexpr double kMinPosition = -10.0; // Min rotations
    constexpr double kSpeed = 20.0;        // Rotations per full joystick input

    // Get current position from leader motor
    double currentPosition = m_elevatorPivot.GetEncoder().GetPosition();
    double targetPosition = currentPosition + (joystickY * kSpeed);

    // Clamp to safe range
    if (targetPosition > kMaxPosition) targetPosition = kMaxPosition;
    if (targetPosition < kMinPosition) targetPosition = kMinPosition;

    // Set position on leader only; follower mimics it
    m_elevatorPivot.GetClosedLoopController().SetReference(
        targetPosition, rev::spark::SparkBase::ControlType::kPosition);
}