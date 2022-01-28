// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Constants.INTAKE;

/** Add your docs here. */
public class Intake {
    /*
    TalonSRX intake = new TalonSRX(INTAKE.intakeCANID);
    TalonSRX rollers = new TalonSRX(INTAKE.rollersCANID);
    
    private Intake()
    {
        intake.configFactoryDefault();
        intake.setNeutralMode(NeutralMode.Brake);
        intake.setInverted(false);
        intake.enableVoltageCompensation(true);
        intake.configVoltageCompSaturation(INTAKE.voltComp);
        intake.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        intake.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

        rollers.configFactoryDefault();
        rollers.setNeutralMode(NeutralMode.Brake);
        rollers.setInverted(false);
        rollers.enableVoltageCompensation(true);
        rollers.configVoltageCompSaturation(INTAKE.voltComp);
    }

    public static Intake getInstance()
    {
        return InstanceHolder.mInstance;
    }

    private static class InstanceHolder
    {
        private static final Intake mInstance = new Intake();
    } 
    */
}
