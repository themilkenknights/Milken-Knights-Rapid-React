// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CLIMBER;

/**The Climber class contains everything relating to the climbing mechanism*/
public class Climber {
    
    TalonFX telescopeArmLeft = new TalonFX(CLIMBER.telescopeArmLeftCANID = 22);
    TalonFX telescopeArmRight = new TalonFX(CLIMBER.telescopeArmRightCANID = 23);

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
    }

    public static Climber getInstance()
    {
        return InstanceHolder.mInstance;
    }

 /**Updates Climber values in shuffleboard*/
    public void climberUpdate()
    {
        SmartDashboard.putNumber("right climb", telescopeArmRight.getSelectedSensorPosition());
        SmartDashboard.putNumber("left climb", telescopeArmLeft.getSelectedSensorPosition());
    }

    /**
     * Powers the telescoping arms at varying speeds [-1, 1]
     * @param percentleft Setpoint of the left motor
     * @param percentright Setpoint of the right motor
     */
    public void telescopePercent(double percentleft, double percentright)
    {
        telescopeArmLeft.set(ControlMode.PercentOutput, percentleft);
        telescopeArmRight.set(ControlMode.PercentOutput, percentright);
    }

    /**
     * Powers the right telescoping arm at varying speeds [-1, 1]
     * @param setpoint Setpoint of the motor
     */
    public void telescopePercentRight(double setpoint)
    {
        telescopeArmRight.set(ControlMode.PercentOutput, setpoint);
    }

    /**
     * Powers the left telescoping arm at varying speeds [-1, 1]
     * @param setpoint Setpoint of the motor
     */
    public void telescopePercentLeft(double setpoint)
    {
        telescopeArmLeft.set(ControlMode.PercentOutput, setpoint);
    }

    /**Zeros the telescoping arms' integrated encoder*/
    public void zeroVClimbb()
    {
        telescopeArmRight.setSelectedSensorPosition(0);
        telescopeArmLeft.setSelectedSensorPosition(0);
    }

    private static class InstanceHolder
    {
        private static final Climber mInstance = new Climber();
    } 
    
}
