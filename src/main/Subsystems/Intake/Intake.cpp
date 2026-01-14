// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Intake.h"

Intake::Intake() {
    intakeMotor.setRotorToSensorRatio(IntakeConstants::IntakeRotorToSensor);
    intakeMotor.setFusedCANCoder(IntakeConstants::IntakeCANCoderId);

    intakeMotor.configureMotionMagic(IntakeConstants::IntakeCruiseVelocity, IntakeConstants::IntakeCruiseAcceleration,
            0.0_tr_per_s_cu);

    frc::SmartDashboard::PutNumber("Intake/TargetIntakeAngle", 0.0);
}

void Intake::setIntakeToAngle(units::degree_t intakeAngle) {
    frc::SmartDashboard::PutNumber("Intake/TargetIntakeAngle", intakeAngle.value());
    intakeMotor.SetControl(intakeVoltage.WithPosition(intakeAngle).WithEnableFOC(true));
}

bool Intake::isIntakeAtPosition(units::degree_t intakeAngle) {
    units::degree_t intakeError = intakeAngle - intakeMotor.GetPosition().GetValue();
    return (units::math::abs(intakeError) < IntakeConstants::IntakeRangeError);
}

void Intake::setRollersVoltage(units::volt_t voltage) {
    rollersMotor.SetControl(rollersVoltage.WithOutput(voltage).WithEnableFOC(true));
}

void Intake::setCenteringVoltage(units::volt_t voltage) {
    centeringMotor.SetControl(centeringVoltage.WithOutput(voltage).WithEnableFOC(true));
}

frc2::CommandPtr Intake::setIntakeCommand(IntakeValues values){
    return frc2::FunctionalCommand(
        [this, values](){
            setIntakeToAngle(values.intake);
            },
        [this, values](){
            setRollersVoltage(values.rollers);
            setCenteringVoltage(values.centering);
        },
        [](bool interrupted){},
        [this, values](){
            return isIntakeAtPosition(values.intake);
        }
    ).ToPtr();
}
// This method will be called once per scheduler run
void Intake::Periodic() {
    frc::SmartDashboard::PutNumber("Intake/CurrentIntakeAngle", intakeMotor.GetPosition().GetValueAsDouble() * 360);
}
