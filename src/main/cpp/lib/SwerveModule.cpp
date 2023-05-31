// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "lib/SwerveModule.h"



SwerveModule::SwerveModule( std::string _name,
                            int _driveCANId,
                            int _steerCANID, 
                            int _canCoderCANID, 
                            bool _steerReversed, 
                            bool _driveReversed, 
                            units::degree_t _steerOffset)
    : name(_name),
      driveMotor{_driveCANId, "CANivore"},
      steerMotor{_steerCANID, "CANivore"},
      steerEncoder{_canCoderCANID, "CANivore"},
      steerOffset{_steerOffset}
    
{
    m_steerPIDController.EnableContinuousInput(-units::radian_t{std::numbers::pi}, units::radian_t{std::numbers::pi});
    driveMotor.SetInverted(_driveReversed);
    steerMotor.SetInverted(_steerReversed);
    
}

frc::SwerveModuleState SwerveModule::GetState() {

    return frc::SwerveModuleState{units::meters_per_second_t{driveMotor.GetVelocity().GetValue().value() * kSwerve::driveMotor_MpT},    // tps * mpt = mps of drive motor
                                  frc::Rotation2d(units::degree_t{steerMotor.GetPosition().GetValue().value() * kSwerve::steerDegPTurn})};   // degrees of steerMotor
}

frc::SwerveModulePosition SwerveModule::GetPostition()
{
    return frc::SwerveModulePosition{units::meter_t{driveMotor.GetPosition().GetValue().value() * kSwerve::driveMotor_MpT},                // meters of drive
                                  frc::Rotation2d(units::degree_t{steerMotor.GetPosition().GetValue().value() * kSwerve::steerDegPTurn})}; // degrees of steerMotor
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState& desiredState)
{
    
    const frc::SwerveModuleState state = frc::SwerveModuleState::Optimize(desiredState, units::radian_t{GetPostition().angle.Radians()});
    
    // Calculate drive output based on current state and desired state
    // This is a velocity PID in meter/sec
    // Wheel speeds have already been desaturated to masSpeed 
    // PID numbers need to be normalized
    units::voltage::volt_t driveOutput = m_drivePIDController.Calculate((driveMotor.GetVelocity().GetValue().value() * kSwerve::driveMotor_MpT), state.speed.value()) * kRobot::maxBatteryVoltage;
    
    // Calculate the feedforward based on desired state (speed)
    // kA, kV need to calculate voltate from speed 
     units::voltage::volt_t driveFeedForward = m_driveFeedforward.Calculate(state.speed);

    // Calculate the steer output based on current state and desired state
    // This is a Position PID
    // PID numbers need to be normalized
    units::voltage::volt_t steerOutput = m_steerPIDController.Calculate(units::radian_t{units::degree_t{steerMotor.GetPosition().GetValue().value() * kSwerve::steerDegPTurn}}, state.angle.Radians()) * kRobot::maxBatteryVoltage;

    // Calculate the steer feed forward based on the steerPID desired velocity
    // kA, kV need to calculate voltate from speed 
    units::voltage::volt_t steerFeedForward = m_steerFeedforward.Calculate(m_steerPIDController.GetSetpoint().velocity);

    // Set the drive voltage
    driveMotor.SetVoltage(driveFeedForward + driveOutput);

    // Set the steer voltage
    steerMotor.SetVoltage(steerFeedForward + steerOutput);
    frc::SmartDashboard::PutNumber(name + "_DriveFF", driveFeedForward.value());
    frc::SmartDashboard::PutNumber(name + "_SteerFF", steerFeedForward.value());
    frc::SmartDashboard::PutNumber(name + "_DrivePID", driveOutput.value());
    frc::SmartDashboard::PutNumber(name + "_SteerPID", steerOutput.value());

}
units::voltage::volt_t SwerveModule::GetSupplyVoltage(){
   
    return (steerMotor.GetSupplyVoltage().GetValue() + driveMotor.GetSupplyVoltage().GetValue() + steerEncoder.GetSupplyVoltage().GetValue()) / 3;
}
