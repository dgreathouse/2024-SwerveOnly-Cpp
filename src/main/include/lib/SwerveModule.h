// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <string.h>
#include <units/angle.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>
#include "ctre/phoenixpro/TalonFX.hpp"
#include "ctre/phoenixpro/CANcoder.hpp"
#include "ctre/phoenixpro/Pigeon2.hpp"
#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
class SwerveModule {
  public:
  SwerveModule(std::string _name, int _driveCANId, int _steerCANID, int _canCoderCANID, bool _steerReversed, bool _driveReversed, units::degree_t _steerOffset);
  frc::SwerveModuleState GetState();
  frc::SwerveModulePosition GetPostition();
  units::voltage::volt_t GetSupplyVoltage();
  units::velocity::meters_per_second_t GetDriveVelocity();
  void SetDesiredState(const frc::SwerveModuleState& state);
  ctre::phoenixpro::controls::VelocityTorqueCurrentFOC m_driveTrqVel = ctre::phoenixpro::controls::VelocityTorqueCurrentFOC{0_tps, 0_A, 0,false}.WithSlot(0);
  ctre::phoenixpro::controls::PositionVoltage m_steerPosVolt = ctre::phoenixpro::controls::PositionVoltage{0_tr}.WithSlot(0).WithEnableFOC(true);

  private:
  
  
  static constexpr units::radians_per_second_t kModuleMaxAngularVelocity = std::numbers::pi * 1_rad_per_s;  // radians per second
  static constexpr units::radians_per_second_squared_t kModuleMaxAngularAcceleration = std::numbers::pi * 2_rad_per_s / 1_s;  // radians per second^2
  std::string name;
  ctre::phoenixpro::hardware::TalonFX driveMotor;
  ctre::phoenixpro::hardware::TalonFX steerMotor;
  ctre::phoenixpro::hardware::CANcoder steerEncoder;
  units::degree_t steerOffset;
  ctre::phoenixpro::controls::VoltageOut driveVoltageOut{0_V};
  ctre::phoenixpro::controls::VoltageOut steerVoltageOut{0_V};

  frc::PIDController m_drivePIDController{0.0,0.0,0};
  frc::SimpleMotorFeedforward<units::meters> m_driveFeedforward{0_V, 4.3_V / 1.5_mps};
  frc::ProfiledPIDController<units::radians> m_steerPIDController{
    0.10,
    0.0,
    0.0,
    {kModuleMaxAngularVelocity,kModuleMaxAngularAcceleration},
    kRobot::Period
  };
  
  frc::SimpleMotorFeedforward<units::radians> m_steerFeedforward{0_V, 0.5_V / 1_rad_per_s};
};
