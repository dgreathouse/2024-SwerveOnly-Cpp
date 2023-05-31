// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "Subsystems/Drivetrain.h"
#include <units/length.h>
#include <units/velocity.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class AutoDrivetrainDriveStraight : public frc2::CommandHelper<frc2::CommandBase, AutoDrivetrainDriveStraight> {
 public:
  AutoDrivetrainDriveStraight( units::length::meter_t distance,  units::velocity::meters_per_second_t speed, Drivetrain& drive);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
private:
  
  Drivetrain* m_drive;  //m_drive is a pointer to Drivetrain class
  units::length::meter_t m_distance;
  units::velocity::meters_per_second_t m_speed;
  bool isFinished = false;

};
