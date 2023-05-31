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
namespace kSwerve {

    /********************* Wheel Constants **************************************/
    constexpr units::meter_t wheelDiameter{0.1029};
    constexpr units::meter_t wheelCircumference{wheelDiameter * std::numbers::pi};

    /********************* Drive Constants **************************************/
    constexpr double driveGearRatio = 7.84615;
    constexpr double driveMotor_MpT = wheelCircumference.value() / driveGearRatio;
    constexpr units::meters_per_second_t driveMaxSpeed{4.381};
    constexpr units::meter_t robotWheelDiameter{0.545};
    constexpr units::meter_t robotWheelCircumference{std::numbers::pi * robotWheelDiameter};
    constexpr double robotWheelMetersPerRadian = robotWheelCircumference.value() / std::numbers::pi * 2.0;
    constexpr units::radians_per_second_t robotMaxAngularVelocity{driveMaxSpeed.value() / robotWheelMetersPerRadian};
    
    /********************* Steer Constants **************************************/
    constexpr double steerGearRatio = 15.42857;
    constexpr double steerDegPTurn = 360.0 / steerGearRatio;// Deg/t Degrees/Motor Turn
}
namespace kRobot {
    constexpr units::voltage::volt_t maxBatteryVoltage{12.5};
    constexpr units::time::millisecond_t Period = 10_ms;
}