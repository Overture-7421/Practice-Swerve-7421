// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"
#include "OvertureLib/Sensors/OverCANCoder/OverCANCoder.h"

struct IntakeValues {

    units::volt_t rollers;
    units::volt_t centering;
    units::degree_t intake;

};

// inline static const std::map<Positions, IntakeValues> IntakePositions = {

    // /*The intake subsystem consists in three motors running at the same time, hence three
    //  variables will be needed, the first volt refers to the centering motor, the secon to the
    //  rollers and finally the degree type variable refers to the pivot*/
    //         {Positions::InitialPosition, {0.0_V, 0.0_V, 30_deg}},
    //         {Positions::Intake, {6.0_V, 10_V, 125_deg}}, //To be defined
    //         {Positions::Through, {1.5_V, 10_V, 125_deg}},
    //         {Positions::IntakeCoralStation, {3.0_V, 8.25_V, 0_deg}}, //To be defined
    //         {Positions::L1Position, {0.0_V, 0.0_V, 30_deg}}, //To be defined
    //       };

struct  IntakeConstants {

    // Intake Positions
    constexpr static const IntakeValues HoldPosition {0.0_V, 0.0_V, 0_deg};
    constexpr static const IntakeValues Intake {6.0_V, 10.0_V, 125_deg};
    constexpr static const IntakeValues Through {1.5_V, 10.0_V, 0_deg};
    constexpr static const IntakeValues L1Position {0.0_V, 0.0_V, 30_deg};
    constexpr static const IntakeValues L1Spit {-6.0_V, 0.0_V, 30_deg};

    constexpr static const units::volt_t RollersSlow = 3.0_V;
    constexpr static const units::volt_t Centering = 8.25_V;

    constexpr static const units::degree_t IntakeRangeError = 3.5_deg;

    constexpr static const units::turns_per_second_t IntakeCruiseVelocity = 50_tps;
    constexpr static const units::turns_per_second_squared_t IntakeCruiseAcceleration = 42_tr_per_s_sq;

    constexpr static const double IntakeRotorToSensor = 77.8461539;

    constexpr static const double IntakeMotorId = 26;
    constexpr static const double IntakeCANCoderId = 27;

    constexpr static const double RollersMotorId = 29;
    constexpr static const double CenteringMotorId = 31;

    constexpr static const OverTalonFXConfig IntakeConfig() { //Limites cuestionables
        OverTalonFXConfig intakeConfig;
        intakeConfig.MotorId = IntakeMotorId;
        intakeConfig.NeutralMode = ControllerNeutralMode::Brake;
        intakeConfig.useFOC = true;
        intakeConfig.Inverted = true;

        intakeConfig.ClosedLoopRampRate = 0.05_s;
        intakeConfig.CurrentLimit = 30_A;
        intakeConfig.StatorCurrentLimit = 120_A;
        intakeConfig.TriggerThreshold = 40_A;
        intakeConfig.TriggerThresholdTime = 0.5_s;
        //intakeConfig.PIDConfigs.GravityType = 1;
        intakeConfig.PIDConfigs.WithKV(1.0).WithKP(47.0);

        return intakeConfig;
    }

    constexpr static const CanCoderConfig IntakeCANConfig() {
        CanCoderConfig intakeCANConfig;
        intakeCANConfig.CanCoderId = IntakeCANCoderId;
        intakeCANConfig.Offset = -0.1640625_tr;
        intakeCANConfig.SensorDirection = ctre::phoenix6::signals::SensorDirectionValue::Clockwise_Positive;

        return intakeCANConfig;
    }

    constexpr static const OverTalonFXConfig RollersConfig() { //Limites cuestionables
        OverTalonFXConfig rollersConfig;
        rollersConfig.MotorId = RollersMotorId;
        rollersConfig.NeutralMode = ControllerNeutralMode::Brake;
        rollersConfig.Inverted = false;

        rollersConfig.CurrentLimit = 30_A;
        rollersConfig.StatorCurrentLimit = 120_A;
        rollersConfig.TriggerThreshold = 40_A;
        rollersConfig.TriggerThresholdTime = 0.5_s;
        rollersConfig.ClosedLoopRampRate = 0.0_s;
        rollersConfig.OpenLoopRampRate = 0.05_s;

        return rollersConfig;
    }

    constexpr static const OverTalonFXConfig CenteringConfig() { //Limites cuestionables
        OverTalonFXConfig centeringConfig;
        centeringConfig.MotorId = CenteringMotorId;
        centeringConfig.NeutralMode = ControllerNeutralMode::Brake;
        centeringConfig.Inverted = false;

        centeringConfig.CurrentLimit = 30_A;
        centeringConfig.StatorCurrentLimit = 120_A;
        centeringConfig.TriggerThreshold = 40_A;
        centeringConfig.TriggerThresholdTime = 0.5_s;
        centeringConfig.ClosedLoopRampRate = 0.0_s;
        centeringConfig.OpenLoopRampRate = 0.05_s;

        return centeringConfig;
    }

};
