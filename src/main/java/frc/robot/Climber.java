// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import frc.robot.Constants.CLIMBER;

/** Add your docs here. */
public class Climber {
    /*
    TalonFX telescopeArmLeft = new TalonFX(CLIMBER.telescopeArmLeftCANID);
    TalonFX telescopeArmRight = new TalonFX(CLIMBER.telescopeArmRightCANID);

    TalonSRX rotateArmLeft = new TalonSRX(CLIMBER.rotateArmLeftCANID);
    TalonSRX rotateArmRight = new TalonSRX(CLIMBER.rotateArmRightCANID);

    private Climber()
    {
        telescopeArmLeft.configFactoryDefault();
        telescopeArmLeft.setNeutralMode(NeutralMode.Brake);
        telescopeArmLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        telescopeArmLeft.setInverted(CLIMBER.telescopeLeftFlipped);
        telescopeArmLeft.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_25Ms);
        telescopeArmLeft.configVelocityMeasurementWindow(CLIMBER.velocityMeasAmount);
        telescopeArmLeft.enableVoltageCompensation(true);
        telescopeArmLeft.configVoltageCompSaturation(CLIMBER.voltComp);
        telescopeArmLeft.config_kP(0, CLIMBER.telescopeKP);
        telescopeArmLeft.config_kI(0, CLIMBER.telescopeKI);
        telescopeArmLeft.config_kD(0, CLIMBER.telescopeKD);
        telescopeArmLeft.config_kF(0, CLIMBER.telescopeKF);
        telescopeArmLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, CLIMBER.statusOneMeas);
        telescopeArmLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, CLIMBER.statusTwoMeas);
        telescopeArmLeft.configMotionSCurveStrength(CLIMBER.telescopeMagicSCurve);
      
        telescopeArmRight.configFactoryDefault();
        telescopeArmRight.setNeutralMode(NeutralMode.Brake);
        telescopeArmRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        telescopeArmRight.setInverted(!CLIMBER.telescopeLeftFlipped);
        telescopeArmRight.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_25Ms);
        telescopeArmRight.configVelocityMeasurementWindow(CLIMBER.velocityMeasAmount);
        telescopeArmRight.enableVoltageCompensation(true);
        telescopeArmRight.configVoltageCompSaturation(CLIMBER.voltComp);
        telescopeArmRight.config_kP(0, CLIMBER.telescopeKP);
        telescopeArmRight.config_kI(0, CLIMBER.telescopeKI);
        telescopeArmRight.config_kD(0, CLIMBER.telescopeKD);
        telescopeArmRight.config_kF(0, CLIMBER.telescopeKF);
        telescopeArmRight.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, CLIMBER.statusOneMeas);
        telescopeArmRight.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, CLIMBER.statusTwoMeas);
        telescopeArmRight.configMotionSCurveStrength(CLIMBER.telescopeMagicSCurve);
        

        rotateArmLeft.configFactoryDefault();
        rotateArmLeft.setNeutralMode(NeutralMode.Brake);
        rotateArmLeft.setInverted(CLIMBER.rotateLeftFlipped);
        rotateArmLeft.enableVoltageCompensation(true);
        rotateArmLeft.configVoltageCompSaturation(CLIMBER.voltComp);
        rotateArmLeft.config_kP(0, CLIMBER.rotateKP);
        rotateArmLeft.config_kI(0, CLIMBER.rotateKI);
        rotateArmLeft.config_kD(0, CLIMBER.rotateKD);
        rotateArmLeft.config_kF(0, CLIMBER.rotateKF);
        rotateArmLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, CLIMBER.statusOneMeas);
        rotateArmLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, CLIMBER.statusTwoMeas);
        rotateArmLeft.configMotionSCurveStrength(CLIMBER.rotateMagicSCurve);
        rotateArmLeft.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_25Ms);
        rotateArmLeft.configVelocityMeasurementWindow(CLIMBER.velocityMeasAmount);
        
        rotateArmRight.configFactoryDefault();
        rotateArmRight.setNeutralMode(NeutralMode.Brake);
        rotateArmRight.setInverted(!CLIMBER.rotateLeftFlipped);
        rotateArmRight.enableVoltageCompensation(true);
        rotateArmRight.configVoltageCompSaturation(CLIMBER.voltComp);
        rotateArmRight.config_kP(0, CLIMBER.rotateKP);
        rotateArmRight.config_kI(0, CLIMBER.rotateKI);
        rotateArmRight.config_kD(0, CLIMBER.rotateKD);
        rotateArmRight.config_kF(0, CLIMBER.rotateKF);
        rotateArmRight.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, CLIMBER.statusOneMeas);
        rotateArmRight.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, CLIMBER.statusTwoMeas);
        rotateArmRight.configMotionSCurveStrength(CLIMBER.rotateMagicSCurve);
        rotateArmRight.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_25Ms);
        rotateArmRight.configVelocityMeasurementWindow(CLIMBER.velocityMeasAmount);
    }

    public static Climber getInstance()
    {
        return InstanceHolder.mInstance;
    }

    public void climberUpdate()
    {

    }

    public void telescopePercent(double percentleft, double percentright)
    {
        telescopeArmLeft.set(ControlMode.PercentOutput, percentleft);
        telescopeArmRight.set(ControlMode.PercentOutput, percentright);
    }

    public void rotatePercent(double percentleft, double percentright)
    {
        rotateArmLeft.set(ControlMode.PercentOutput, percentleft);
        rotateArmRight.set(ControlMode.PercentOutput, percentright);
    }

    private static class InstanceHolder
    {
        private static final Climber mInstance = new Climber();
    } 
    */
}
