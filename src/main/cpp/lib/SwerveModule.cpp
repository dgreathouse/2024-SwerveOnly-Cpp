// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "lib/SwerveModule.h"
#include "ctre/phoenixpro/TalonFX.hpp"


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
    ctre::phoenixpro::configs::TalonFXConfiguration driveConfig{};
    driveConfig.Slot0.kP = 5; // An error of 1 rotation per second results in 5 amps output
    driveConfig.Slot1.kI = 0.1; // An error of 1 rotation per second increases output by 0.1 amps every second
    driveConfig.Slot1.kD = 0.001; // A change of 1000 rotation per second squared results in 1 amp output
    driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;  // Peak output of 40 amps
    driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;  // Peak output of 40 amps
    driveMotor.GetConfigurator().Apply(driveConfig); 

    ctre::phoenixpro::configs::TalonFXConfiguration steerConfig{};
    steerConfig.Slot0.kP = 24; // An error of 0.5 rotations results in 12 V output
    steerConfig.Slot0.kI = 0.0;  
    steerConfig.Slot0.kD = 0.1; // A velocity of 1 rps results in 0.1 V output
    steerMotor.GetConfigurator().Apply(steerConfig);

}
units::velocity::meters_per_second_t SwerveModule::GetDriveVelocity()
{
    return units::velocity::meters_per_second_t{driveMotor.GetVelocity().GetValue().value() * kSwerve::driveMotor_MpT};
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
    
    /** Calculate the PID output volts with actual speed vs requested speed in Mps.
     *  Normalized +/- maxSpeed is used with a PID to give +/- 1 to multiply by voltage
     *  Therefore driveOutput should never be > 12.5
     *  If state and motor velocity are equal the output is 0
     *  A feedforward is used to get the motor to the needed speed.
     *  This output will maintain the actual speed with the PID calculation.
    */
   // units::voltage::volt_t driveOutput = m_drivePIDController.Calculate((driveMotor.GetVelocity().GetValue().value() * kSwerve::driveMotor_MpT), state.speed.value()) * kRobot::maxBatteryVoltage;
    
    /** Calculate the feedforward based on desired state (speed)
     kS, kV, kA=0 need to calculate voltate from speed if using a simpleMotorFeedForward.
     kV is basically a scale factor of desired speed to voltage.
     kS is used to overcome static friction. kS value is always added and therefore may make the wheel move when tuned close to limit
     kS is not used since this is also used with a PID loop we will rely on the PID to overcome friction
     kA is an accelleration scale to help the wheel. This is not used since it has no feedback.
     Since kS and kA are not used this becomes ( volts = mps * volts/mps; )
     A feedforward value is contant value that will be sent to the motor based on the requested speed.
     Ex: If we know that it takes 2.5 volts to get our robot running at 1 mps the when we want to go at 2 mps we will give the motor 5 volts.
     This scale factor of volts/mps is tested value and is not perfect for every operating condition. Therefore the PID is calculated to 
     get the motor to the perfect speed. The FeedForward will do most of the work to get the motor to goto the speed. The PID needs to maintain the speed.
     If the feedforward actual speed is higher than pid speed then the PID will lower the output which will lower the speed
    */
     //units::voltage::volt_t driveFeedForward = m_driveFeedforward.Calculate(state.speed);
    // units::voltage::volt_t driveFeedForward = units::voltage::volt_t{state.speed.value() * kSwerve::driveVpMps.m_value()};

    // Calculate the steer output based on current state and desired state
    // This is a Position PID
    // PID numbers need to be normalized
   // units::voltage::volt_t steerOutput = m_steerPIDController.Calculate(units::radian_t{units::degree_t{steerMotor.GetPosition().GetValue().value() * kSwerve::steerDegPTurn}}, state.angle.Radians()) * kRobot::maxBatteryVoltage;

    // Calculate the steer feed forward based on the steerPID desired velocity
    // kA, kV need to calculate voltate from speed 
   // units::voltage::volt_t steerFeedForward = m_steerFeedforward.Calculate(m_steerPIDController.GetSetpoint().velocity);
    // Set the steer position
    units::turn_t steerPos{state.angle.Degrees().value() * kSwerve::steerTpDeg};
    steerMotor.SetControl(m_steerPosVolt.WithPosition(steerPos));


    // Set the drive velocity
    units::turns_per_second_t driveSpeed{state.speed.value() * kSwerve::driveMotor_TpM};
    driveMotor.SetControl(m_driveTrqVel.WithVelocity(driveSpeed));
    //driveMotor.SetControl(driveVoltageOut.WithEnableFOC(true).WithOverrideBrakeDurNeutral(false).WithOutput(driveFeedForward + driveOutput));
    
    // Set the steer voltage
    //steerMotor.SetControl(steerVoltageOut.WithEnableFOC(true).WithOverrideBrakeDurNeutral(false).WithOutput(steerFeedForward + steerOutput));

    // frc::SmartDashboard::PutNumber(name + "_DriveFF", driveFeedForward.value());
    // frc::SmartDashboard::PutNumber(name + "_DrivePID", driveOutput.value());
    frc::SmartDashboard::PutNumber(name + "_DriveVel", GetDriveVelocity().value());
    frc::SmartDashboard::PutNumber(name + "_DriveStateVelMps", state.speed.value());
    
   // frc::SmartDashboard::PutNumber(name + "_SteerFF", steerFeedForward.value());
  //  frc::SmartDashboard::PutNumber(name + "_SteerPID", steerOutput.value());
    frc::SmartDashboard::PutNumber(name + "_SteerStateAngDeg", state.angle.Degrees().value());
    frc::SmartDashboard::PutNumber(name + "_SteerEncAngDeg", steerEncoder.GetPosition().GetValue().value() * 360.0);
    frc::SmartDashboard::PutNumber(name + "_SteerMotAngDeg", steerMotor.GetPosition().GetValue().value() * 360.0);
}
units::voltage::volt_t SwerveModule::GetSupplyVoltage(){
   
    return (steerMotor.GetSupplyVoltage().GetValue() + driveMotor.GetSupplyVoltage().GetValue() + steerEncoder.GetSupplyVoltage().GetValue()) / 3;
}
