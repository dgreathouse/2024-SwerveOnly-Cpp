// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "lib/SwerveModule.h"
#include "ctre/phoenixpro/Pigeon2.hpp"

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/geometry/Pose2d.h>
#include <units/velocity.h>

class Drivetrain : public frc2::SubsystemBase {
 public:
  
  Drivetrain();
  void Drive(units::meters_per_second_t _xSpeed, units::meters_per_second_t _ySpeed, units::radians_per_second_t _rot, bool _fieldRelative);
  void UpdateOdometry();
  units::radians_per_second_t CalcRobotAngle(units::degree_t _desiredAngle, units::degree_t _currentAngle);
  bool BusActive();
  void Periodic() override;
  frc::Pose2d GetRobotPose();
 private:
  frc::Translation2d m_frontLeftLocation{+0.170_m, +0.235_m};
  frc::Translation2d m_frontRightLocation{+0.170_m, -0.235_m};
  frc::Translation2d m_backLocation{-0.170_m, 0.0_m};

  SwerveModule fl;//{1,1,1,false,false,130.0_deg};
  SwerveModule fr;//{1,1,1,false,false,130.0_deg};
  SwerveModule b;//{1,1,1,false,false,130.0_deg};

  ctre::phoenixpro::hardware::Pigeon2 gyro;//{1, "CANivore"};

  frc::SwerveDriveKinematics<3U> m_kinematics;//{m_frontLeftLocation, m_frontRightLocation, m_backLocation};
  frc::SwerveDriveOdometry<3U> m_odometery;//{ m_kinematics, gyro.GetRotation2d(), {fl.GetPostition(), fr.GetPostition(), b.GetPostition()}};

  static constexpr units::radians_per_second_t kRobotMaxAngularVelocity = std::numbers::pi * 1_rad_per_s;  // radians per second
  static constexpr units::radians_per_second_squared_t kRobotMaxAngularAcceleration = std::numbers::pi * 2_rad_per_s / 1_s;  // radians per second^2

  frc::ProfiledPIDController<units::radians> m_robotAnglePIDController{
    3.50,
    1.50,
    0.0,
    {kRobotMaxAngularVelocity,kRobotMaxAngularAcceleration},
    kRobot::Period
  };
  
  frc::SimpleMotorFeedforward<units::radians> m_robotAngleFeedforward{0_V, 1.5_V / 1_rad_per_s};
};
