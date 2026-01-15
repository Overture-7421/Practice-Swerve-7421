// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/MathUtil.h>
#include <frc2/command/FunctionalCommand.h>
#include <ctre/phoenix6/CANrange.hpp>
#include "Subsystems/Intake/IntakeConstants.h"
#include "Constants.h"

class Intake: public frc2::SubsystemBase {
public:
    Intake();

    void setIntakeToAngle(units::degree_t intakeAngle);
    bool isIntakeAtPosition(units::degree_t intakeAngle);

    void setRollersVoltage(units::volt_t voltage);
    void setCenteringVoltage(units::volt_t voltage);

    frc2::CommandPtr setIntakeCommand(IntakeValues values);

    void Periodic() override;

private:

    OverTalonFX intakeMotor {IntakeConstants::IntakeConfig(), robotConstants::rio};
    OverCANCoder intakeCANCoder {IntakeConstants::IntakeCANConfig(), robotConstants::rio};

    OverTalonFX rollersMotor {IntakeConstants::RollersConfig(), robotConstants::rio};
    OverTalonFX centeringMotor {IntakeConstants::CenteringConfig(), robotConstants::rio};

    ctre::phoenix6::controls::MotionMagicVoltage intakeVoltage {0_tr};
    ctre::phoenix6::controls::VoltageOut rollersVoltage {0_V};
    ctre::phoenix6::controls::VoltageOut centeringVoltage {0_V};

};
