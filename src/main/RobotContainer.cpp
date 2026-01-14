// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

RobotContainer::RobotContainer() {
  autoChooser = pathplanner::AutoBuilder::buildAutoChooser();
	frc::SmartDashboard::PutData("AutoChooser", &autoChooser);

  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  ConfigDriverBindings();
}

void RobotContainer::ConfigDriverBindings() {
    chassis.SetDefaultCommand(DriveCommand(&chassis, &driver).ToPtr());
    driver.Back().OnTrue(ResetHeading(&chassis));

    driver.LeftTrigger().WhileTrue(intake.setIntakeCommand(IntakeConstants::Intake));
    driver.LeftTrigger().OnFalse(frc2::cmd::Sequence(
      intake.setIntakeCommand(IntakeConstants::Through),
      intake.setIntakeCommand(IntakeConstants::HoldPosition))
    );

    driver.RightTrigger().WhileTrue(intake.setIntakeCommand(IntakeConstants::L1Position));
    driver.RightTrigger().OnFalse(intake.setIntakeCommand(IntakeConstants::HoldPosition));

    driver.RightBumper().WhileTrue(intake.setIntakeCommand(IntakeConstants::L1Spit));
    driver.RightBumper().OnFalse(intake.setIntakeCommand(IntakeConstants::HoldPosition));

}

frc2::Command* RobotContainer::GetAutonomousCommand() {

	return autoChooser.GetSelected();
}