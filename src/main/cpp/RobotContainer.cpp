// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>
#include "commands/DrivetrainDefaultCommand.h"


RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  

  m_autoChooser.SetDefaultOption("Do Nothing", &m_autoDoNothing);
  m_autoChooser.AddOption("Test", &m_autoTest);

  frc::SmartDashboard::PutData(&m_autoChooser);

  DrivetrainDefaultCommand drivetrainDefaultCommand = DrivetrainDefaultCommand(drivetrain,driverStick);
  drivetrain.SetDefaultCommand(drivetrainDefaultCommand);


  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here

  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.

}
frc2::Command* RobotContainer::GetAutonomousCommand() {
  // Runs the chosen command in autonomous
  return m_autoChooser.GetSelected();
}


