// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/Drivetrain.h"
#include <frc/Joystick.h>
#include <frc/filter/SlewRateLimiter.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class DrivetrainDefaultCommand
    : public frc2::CommandHelper<frc2::CommandBase, DrivetrainDefaultCommand> {
 public:
  DrivetrainDefaultCommand(Drivetrain& drive, frc::Joystick& driverStick);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
 Drivetrain* m_drivetrain;
 frc::Joystick* m_driverStick;


 frc::SlewRateLimiter<units::scalar> m_xSpeedSRLimiter{kSwerve::driveMaxSpeed.value()/2.0_s};
 frc::SlewRateLimiter<units::scalar> m_ySpeedSRLimiter{kSwerve::driveMaxSpeed.value()/2.0_s};
 frc::SlewRateLimiter<units::scalar> m_rotSpeedSRLimiter{kSwerve::robotMaxAngularVelocity.value() /2.0_s};


};
