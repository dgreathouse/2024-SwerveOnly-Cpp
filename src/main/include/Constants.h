// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */
#include <units/velocity.h>
#include <units/length.h>
#include <numbers>

namespace OperatorConstants {

    constexpr int kDriverControllerPort = 0;

}  // namespace OperatorConstants
namespace kRobot {
    constexpr units::voltage::volt_t maxBatteryVoltage{12.5};
    constexpr units::time::millisecond_t Period = 20_ms;

    constexpr double xStickMax = 0.866;
    constexpr double xStickMin = -0.82;

    constexpr double yStickMax = 0.646;
    constexpr double yStickMin = -0.7;

    constexpr double rotStickMax = 0.811;
    constexpr double rotStickMin = -0.875;

}
namespace kSwerve {

    /********************* Wheel Constants **************************************/
    constexpr units::meter_t wheelDiameter{0.1029};
    constexpr units::meter_t wheelCircumference{wheelDiameter * std::numbers::pi};

    /********************* Drive Constants **************************************/
    constexpr double driveGearRatio = 7.84615;
    constexpr double driveMotor_MpT = wheelCircumference.value() / driveGearRatio;
    
    constexpr double driveMotor_TpM = 1.0/driveMotor_MpT;
    constexpr units::meters_per_second_t driveMaxSpeed{4.381};
    constexpr units::meter_t robotWheelDiameter{0.545};
    constexpr units::meter_t robotWheelCircumference{std::numbers::pi * robotWheelDiameter};
    constexpr double robotWheelMetersPerRadian = robotWheelCircumference.value() / std::numbers::pi * 2.0;
    constexpr units::radians_per_second_t robotMaxAngularVelocity{driveMaxSpeed.value() / robotWheelMetersPerRadian};
    constexpr units::linear_scale driveVpMps = 4.3_V/1.5_mps;
   // constexpr units::linear_scale driveVpMpsd = kRobot::maxBatteryVoltage/kSwerve::driveMaxSpeed;
    /********************* Steer Constants **************************************/
    constexpr double steerGearRatio = 15.42857;
    constexpr double steerDegPTurn = 360.0 / steerGearRatio;// Deg/t Degrees/Motor Turn
    constexpr double steerTpDeg = steerGearRatio / 360.0;// turn/deg Degrees/Motor Turn
}
