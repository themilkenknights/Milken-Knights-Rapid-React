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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SHOOT;

/** Add your docs here. */
public class Shooter {
    
    TalonFX shootLeft = new TalonFX(SHOOT.shootLeftCANID);
    TalonFX shootRight = new TalonFX(SHOOT.shootRightCANID);
    TalonSRX belt = new TalonSRX(SHOOT.beltCANID);
    
    private Shooter()
    {
        shootLeft.configFactoryDefault();
        shootLeft.setNeutralMode(NeutralMode.Coast);
        shootLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        shootLeft.setInverted(SHOOT.leftFlipped);
        shootLeft.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_25Ms);
        shootLeft.configVelocityMeasurementWindow(SHOOT.velocityMeasAmount);
        shootLeft.enableVoltageCompensation(true);
        shootLeft.configVoltageCompSaturation(SHOOT.voltComp);
        shootLeft.config_kP(0, SHOOT.shootKP);
        shootLeft.config_kI(0, SHOOT.shootKI);
        shootLeft.config_kD(0, SHOOT.shootKD);
        shootLeft.config_kF(0, SHOOT.shootKF);
        shootLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, SHOOT.statusOneMeas);
        shootLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, SHOOT.statusTwoMeas);

        

        shootRight.configFactoryDefault();
        shootRight.setNeutralMode(NeutralMode.Coast);
        shootRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        shootRight.setInverted(!SHOOT.leftFlipped);
        shootRight.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_25Ms);
        shootRight.configVelocityMeasurementWindow(SHOOT.velocityMeasAmount);
        shootRight.enableVoltageCompensation(true);
        shootRight.configVoltageCompSaturation(SHOOT.voltComp);
        shootRight.config_kP(0, SHOOT.shootKP);
        shootRight.config_kI(0, SHOOT.shootKI);
        shootRight.config_kD(0, SHOOT.shootKD);
        shootRight.config_kF(0, SHOOT.shootKF);
        shootRight.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, SHOOT.statusOneMeas);
        shootRight.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, SHOOT.statusTwoMeas);



        belt.configFactoryDefault();
        belt.setNeutralMode(NeutralMode.Brake);
        belt.setInverted(false);
        belt.enableVoltageCompensation(true);
        belt.configVoltageCompSaturation(SHOOT.voltComp);
    }
    

    public static Shooter getInstance()
    {
        return InstanceHolder.mInstance;
    }
    
    public void shooterUpdate()
    {
        SmartDashboard.putNumber("leftSpeed", shootLeft.getSelectedSensorVelocity());
        SmartDashboard.putNumber("rightSpeed", shootRight.getSelectedSensorVelocity());
    }

    public void setShooterPercent(double percent)
    {
        shootLeft.set(ControlMode.PercentOutput, percent);
        shootRight.set(ControlMode.PercentOutput, percent);
    }

    public void setShooterNativeVeloctiy(double setpoint)
    {
        shootLeft.set(ControlMode.Velocity, setpoint);
        shootRight.set(ControlMode.Velocity, setpoint);
    }

    public void setShooterNativeVeloCalc(double setpoint)
    {
         shootLeft.set(ControlMode.Velocity, setpoint + shooterFeedForward(setpoint));
         shootRight.set(ControlMode.Velocity, setpoint + shooterFeedForward(setpoint));
    }

    public double shooterFeedForward(double setpoint)
    {
        return -SHOOT.maxError * (Math.cos((Constants.kPi / 2) * (setpoint / SHOOT.maxVelo)));
    }

    private static class InstanceHolder
    {
        private static final Shooter mInstance = new Shooter();
    } 
    
}
