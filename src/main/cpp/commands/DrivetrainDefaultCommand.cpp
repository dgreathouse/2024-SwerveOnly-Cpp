// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DrivetrainDefaultCommand.h"
#include "RobotContainer.h"
#include "Constants.h"
#include "units/velocity.h"
#include "units/angular_velocity.h"

DrivetrainDefaultCommand::DrivetrainDefaultCommand(Drivetrain& drive, frc::Joystick& driverStick) 
    : m_drivetrain(&drive),
      m_driverStick(&driverStick){
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_drivetrain);
}

// Called when the command is initially scheduled.
void DrivetrainDefaultCommand::Initialize() {
  //RobotContainer::drive.GetCurrentCommand();
}

// Called repeatedly when this Command is scheduled to run
void DrivetrainDefaultCommand::Execute() {

  const units::meters_per_second_t xSpeed = -m_xSpeedLimiter.Calculate(frc::ApplyDeadband(m_driverStick->GetRawAxis(0), 0.02)) * kSwerve::driveMaxSpeed;
  const units::meters_per_second_t ySpeed = -m_ySpeedLimiter.Calculate(frc::ApplyDeadband(m_driverStick->GetRawAxis(1), 0.02)) * kSwerve::driveMaxSpeed;
  const units::radians_per_second_t rot = -m_rotSpeedLimiter.Calculate(frc::ApplyDeadband(m_driverStick->GetRawAxis(2), 0.02)) * kSwerve::robotMaxAngularVelocity;

  
  m_drivetrain->Drive(xSpeed,ySpeed,rot, true);

}

// Called once the command ends or is interrupted.
void DrivetrainDefaultCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool DrivetrainDefaultCommand::IsFinished() {
  return false;
}
