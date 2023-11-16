// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoDrivetrainDriveStraight.h"

AutoDrivetrainDriveStraight::AutoDrivetrainDriveStraight( units::length::meter_t _distance,  units::velocity::meters_per_second_t _speed, Drivetrain& _drive)
  : m_drive(&_drive),
    m_distance(_distance),
    m_speed(_speed)


 {
  AddRequirements(m_drive);
  
}

// Called when the command is initially scheduled.
void AutoDrivetrainDriveStraight::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void AutoDrivetrainDriveStraight::Execute() {
  if(m_drive->GetRobotPose().X().operator>=(m_distance)){
    isFinished = true;
    m_drive->Drive(0_mps, 0_mps, 0_rad_per_s,true,true);
  }else {
    m_drive->Drive(m_speed, 0_mps, 0_rad_per_s,true,true);
  }
 // const char* s = m_speed.abbreviation();
  // if( m_drive->GetRobotPose().X().value() > m_distance.value()) {
  //   isFinished = true;
  // }
}

// Called once the command ends or is interrupted.
void AutoDrivetrainDriveStraight::End(bool interrupted) {}

// Returns true when the command should end.
bool AutoDrivetrainDriveStraight::IsFinished() {
  return isFinished;
}
