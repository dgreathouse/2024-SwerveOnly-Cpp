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
  bool BusActive();
  void Periodic() override;
  frc::Pose2d GetRobotPose();
 private:
  frc::Translation2d m_frontLeftLocation{+0.135_m, +0.235_m};
  frc::Translation2d m_frontRightLocation{+0.135_m, -0.235_m};
  frc::Translation2d m_backLocation{-0.272_m, 0.0_m};

  SwerveModule fl;//{1,1,1,false,false,130.0_deg};
  SwerveModule fr;//{1,1,1,false,false,130.0_deg};
  SwerveModule b;//{1,1,1,false,false,130.0_deg};

  ctre::phoenixpro::hardware::Pigeon2 gyro;//{1, "CANivore"};

  frc::SwerveDriveKinematics<3U> m_kinematics;//{m_frontLeftLocation, m_frontRightLocation, m_backLocation};
  frc::SwerveDriveOdometry<3U> m_odometery;//{ m_kinematics, gyro.GetRotation2d(), {fl.GetPostition(), fr.GetPostition(), b.GetPostition()}};
  
};
