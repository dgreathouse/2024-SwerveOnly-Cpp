// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivetrain.h"
#include <frc/kinematics/SwerveModulePosition.h>
#include "Constants.h"
#include "units/length.h"



  Drivetrain::Drivetrain() 
   :  fl{"fl",13,23,3,true,false,151.857_deg},
      fr{"fr",12,22,2,true,true,286.523_deg},
      b{"b",11,21,1,true,false,36.47_deg},
      gyro{5,"CANivore"},
      m_kinematics{m_frontLeftLocation, m_frontRightLocation, m_backLocation},
      m_odometery{ m_kinematics, gyro.GetRotation2d(), {fl.GetPostition(), fr.GetPostition(), b.GetPostition()}}
  {
      while(!BusActive()){}
      gyro.Reset();
  }

// This method will be called once per scheduler run
void Drivetrain::Periodic() {
    UpdateOdometry();
    //frc::SmartDashboard::PutNumber("RobotAngle", gyro.GetAngle());
}
bool Drivetrain::BusActive(){
   if(fl.GetSupplyVoltage().value() > 4.5 && fr.GetSupplyVoltage().value() > 4.5 && b.GetSupplyVoltage().value() > 4.5){
      return true;
   }
   return false;
}
void Drivetrain::Drive(units::meters_per_second_t _xSpeed, units::meters_per_second_t _ySpeed, units::radians_per_second_t _rotSpeed, bool _fieldRelative){
   units::radians_per_second_t rot = CalcRobotAngle(45_deg, gyro.GetYaw().GetValue());
   wpi::array<frc::SwerveModuleState, 3U> states = m_kinematics.ToSwerveModuleStates(
      _fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(_xSpeed, _ySpeed, rot, gyro.GetRotation2d())
                     : frc::ChassisSpeeds(_xSpeed, _ySpeed, rot));

   m_kinematics.DesaturateWheelSpeeds(&states, kSwerve::driveMaxSpeed);

   auto [flState,frState,bState] = states;

   fl.SetDesiredState(flState);
   fr.SetDesiredState(frState);
   b.SetDesiredState(bState);
 
}
void Drivetrain::UpdateOdometry() {
   m_odometery.Update(gyro.GetRotation2d(),{fl.GetPostition(), fr.GetPostition(), b.GetPostition()});
   frc::SmartDashboard::PutNumber("X_Pose", m_odometery.GetPose().X().value());
   frc::SmartDashboard::PutNumber("Y_Pose", m_odometery.GetPose().Y().value());
   frc::SmartDashboard::PutNumber("RobotAngle", gyro.GetYaw().GetValue().value());

}
units::radians_per_second_t Drivetrain::CalcRobotAngle(units::degree_t _desiredAngle, units::degree_t _currentAngle)
{
   double pidOutput = -m_robotAnglePIDController.Calculate(_desiredAngle, _currentAngle);
   double pidFF = -m_robotAngleFeedforward.Calculate(m_robotAnglePIDController.GetSetpoint().velocity).value();
   frc::SmartDashboard::PutNumber("RobotAnglePIDOutput", pidOutput);
   frc::SmartDashboard::PutNumber("RobotAngleFF", pidFF);
   return units::radians_per_second_t{pidOutput + pidFF};
}

frc::Pose2d Drivetrain::GetRobotPose(){
   return m_odometery.GetPose();
}
