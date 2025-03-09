#include "RobotContainer.h"
#include <frc/geometry/Rotation2d.h>
#include <thread>
#include <chrono>

RobotContainer::RobotContainer()
    : m_navx(std::make_unique<studica::AHRS>(studica::AHRS::NavXComType::kMXP_SPI, 200)),
      m_odometry(m_kinematics, 
                 frc::Rotation2d{0_rad}, 
                 {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                  m_backLeft.GetPosition(), m_backRight.GetPosition()},
                 frc::Pose2d{}) {
    int attempts = 0;
    const int maxAttempts = 250;
    while (!m_navx->IsConnected() && attempts < maxAttempts) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        attempts++;
    }

    if (m_navx->IsConnected()) {
        while (m_navx->IsCalibrating()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        m_navx->ZeroYaw();
    } 

    rev::spark::SparkBaseConfig leaderConfig{};
    leaderConfig
        .Inverted(false)
        .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake)
        .closedLoop
            .SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
            .Pid(0.1, 0.0, 0.0)
        .OutputRange(-0.5, 0.5);
    m_elevatorPivot.Configure(leaderConfig, rev::spark::SparkBase::ResetMode::kResetSafeParameters,
                              rev::spark::SparkBase::PersistMode::kPersistParameters);

    rev::spark::SparkBaseConfig followerConfig{};
    followerConfig
        .Inverted(true)
        .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake)
        .Follow(m_elevatorPivot.GetDeviceId(), true);
    m_elevatorPivot2.Configure(followerConfig, rev::spark::SparkBase::ResetMode::kResetSafeParameters,
                               rev::spark::SparkBase::PersistMode::kPersistParameters);
}

void RobotContainer::Drive(double xSpeed, double ySpeed, double rot, bool fieldRelative) {
    if (std::abs(xSpeed) < OperatorConstants::kDeadband) xSpeed = 0.0;
    if (std::abs(ySpeed) < OperatorConstants::kDeadband) ySpeed = 0.0;
    if (std::abs(rot) < OperatorConstants::kDeadband) rot = 0.0;

    if (xSpeed == 0.0 && ySpeed == 0.0 && rot == 0.0) {
        m_frontLeft.SetDesiredState({0_mps, m_frontLeft.GetState().angle});
        m_frontRight.SetDesiredState({0_mps, m_frontRight.GetState().angle});
        m_backLeft.SetDesiredState({0_mps, m_backLeft.GetState().angle});
        m_backRight.SetDesiredState({0_mps, m_backRight.GetState().angle});
        return;
    }

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

    auto states = m_kinematics.ToSwerveModuleStates(speeds);
    m_kinematics.DesaturateWheelSpeeds(&states, AutoConstants::kMaxSpeed);

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
    constexpr double kMaxPosition = 10.0;
    constexpr double kMinPosition = -10.0;
    constexpr double kDeadband = 0.1;

    if (std::abs(joystickY) < kDeadband) {
        return;
    }

    double targetPosition = joystickY * kMaxPosition;
    targetPosition = std::clamp(targetPosition, kMinPosition, kMaxPosition);

    m_elevatorPivot.GetClosedLoopController().SetReference(
        targetPosition, rev::spark::SparkBase::ControlType::kPosition);
}

frc::Pose2d RobotContainer::GetPose() const {
    return m_odometry.GetPose(); 
}