#pragma once

#include "SwerveModule.h"
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/geometry/Translation2d.h>
#include <studica/AHRS.h> // Studica navX2 library (commented out for now)
#include <frc/SPI.h>      // For SPI::Port
#include <units/length.h> // For units::foot_t
#include <array>

class Drivetrain {
 public:
  Drivetrain();
  void Drive(units::feet_per_second_t xSpeed, units::feet_per_second_t ySpeed, units::radians_per_second_t rotSpeed);
  frc::Pose2d GetPose() const;
  void UpdateOdometry();

 private:
  static constexpr double kSwerveAxisDistance = 23.5 / 12.0; // 23.5 inches to feet
  static constexpr units::feet_per_second_t kMaxSpeed = 15.0_fps;
  static constexpr units::radians_per_second_t kMaxAngularSpeed = 2 * M_PI * 1_rad / 1_s;

  const frc::Translation2d m_flLocation{units::foot_t(kSwerveAxisDistance / 2), units::foot_t(kSwerveAxisDistance / 2)};
  const frc::Translation2d m_frLocation{units::foot_t(kSwerveAxisDistance / 2), units::foot_t(-kSwerveAxisDistance / 2)};
  const frc::Translation2d m_blLocation{units::foot_t(-kSwerveAxisDistance / 2), units::foot_t(kSwerveAxisDistance / 2)};
  const frc::Translation2d m_brLocation{units::foot_t(-kSwerveAxisDistance / 2), units::foot_t(-kSwerveAxisDistance / 2)};

  SwerveModule m_frontLeft{5, 1, 9, 0.0};   // FL: Drive 5, Steer 1, CANcoder 9
  SwerveModule m_frontRight{6, 2, 10, 0.0}; // FR: Drive 6, Steer 2, CANcoder 10
  SwerveModule m_backLeft{8, 4, 12, 0.0};   // BL: Drive 8, Steer 4, CANcoder 12
  SwerveModule m_backRight{7, 3, 11, 0.0};  // BR: Drive 7, Steer 3, CANcoder 11

  double m_dummyHeading = 0.0; // Fallback until AHRS constructor is resolved
  frc::SwerveDriveKinematics<4> m_kinematics{m_flLocation, m_frLocation, m_blLocation, m_brLocation};
  const std::array<frc::SwerveModulePosition, 4> m_initialPositions; // Made const, initialized in constructor
  frc::SwerveDriveOdometry<4> m_odometry; // Initialized in constructor with m_initialPositions
};