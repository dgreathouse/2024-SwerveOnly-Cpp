// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DrivetrainDefaultCommand.h"
#include "RobotContainer.h"
#include "Constants.h"
#include "units/velocity.h"
#include "units/angular_velocity.h"
#include <lib/StickLinear.h>
DrivetrainDefaultCommand::DrivetrainDefaultCommand(Drivetrain &drive, frc::Joystick &driverStick)
    : m_drivetrain(&drive),
      m_driverStick(&driverStick)
{
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_drivetrain);
}

// Called when the command is initially scheduled.
void DrivetrainDefaultCommand::Initialize()
{
  // RobotContainer::drive.GetCurrentCommand();
  // m_xSpeedSRLimiter.Reset(0.0);
  // m_ySpeedSRLimiter.Reset(0.0);
  // m_rotSpeedSRLimiter.Reset(0.0);
}

// Called repeatedly when this Command is scheduled to run
void DrivetrainDefaultCommand::Execute()
{
  double x = StickLinear::Linearize(m_driverStick->GetRawAxis(0), kRobot::xStickMin, kRobot::xStickMax, -0.0315);
  double y = StickLinear::Linearize(m_driverStick->GetRawAxis(1), kRobot::yStickMin, kRobot::yStickMax, -0.09);
  double rot = StickLinear::Linearize(m_driverStick->GetRawAxis(3), kRobot::rotStickMin, kRobot::rotStickMax, -0.07);

  frc::SmartDashboard::PutNumber("X_StickLin", x);
  frc::SmartDashboard::PutNumber("y_StickLin", y);
  frc::SmartDashboard::PutNumber("rot_StickLin", rot);

   units::scalar_t xS{frc::ApplyDeadband(x, 0.02) * kSwerve::driveMaxSpeed.value()};
   units::scalar_t yS{frc::ApplyDeadband(y, 0.02) * kSwerve::driveMaxSpeed.value()};
   units::scalar_t rS{frc::ApplyDeadband(rot, 0.02) * kSwerve::robotMaxAngularVelocity.value()};

   frc::SmartDashboard::PutNumber("X_StickDbMps", xS.value());
   frc::SmartDashboard::PutNumber("Y_StickDbMps", yS.value());
   frc::SmartDashboard::PutNumber("rot_StickDbMps", rS.value());

   units::meters_per_second_t xSpeed = units::meters_per_second_t{-m_xSpeedSRLimiter.Calculate(xS).value()};
   units::meters_per_second_t ySpeed = units::meters_per_second_t{-m_ySpeedSRLimiter.Calculate(yS).value()};
   units::radians_per_second_t rotRate = units::radians_per_second_t{-m_rotSpeedSRLimiter.Calculate(rS).value()};

   frc::SmartDashboard::PutNumber("X_StickSRL", xSpeed.value());
   frc::SmartDashboard::PutNumber("Y_StickSRL", ySpeed.value());
   frc::SmartDashboard::PutNumber("rot_StickSRL", rotRate.value());

//  m_drivetrain->Drive(xSpeed, ySpeed, rotRate, true);
  m_drivetrain->Drive(0_mps, 0_mps, 0_rad_per_s, true);
}

// Called once the command ends or is interrupted.
void DrivetrainDefaultCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool DrivetrainDefaultCommand::IsFinished()
{
  return false;
}
