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
import frc.robot.Constants.SHOOT;

/**bang*/
public class Shooter {
    
     TalonFX shootLeft = new TalonFX(SHOOT.shootLeftCANID);
     TalonFX shootRight = new TalonFX(SHOOT.shootRightCANID);
     private double avgShootSpeed = 0;
     private double leftShootVelocity = 0;
     private double rightShootVelocity = 0;
    
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
    }
    

    public static Shooter getInstance()
    {
        return InstanceHolder.mInstance;
    }
    
    public void shooterUpdate()
    {
        avgShootSpeed = (getShootLeftVelocity() + getShootRightVelocity()) / 2;
        leftShootVelocity = getShootLeftVelocity();
        rightShootVelocity = getShootRightVelocity();
    }

    public void updateShooterVelocity()
    {
        SmartDashboard.putNumber("leftSpeed", shootLeft.getSelectedSensorVelocity());
        SmartDashboard.putNumber("rightSpeed", shootRight.getSelectedSensorVelocity());
    }

    public void setShooterPercent(double setpoint)
    {
        shootLeft.set(ControlMode.PercentOutput, setpoint);
        shootRight.set(ControlMode.PercentOutput, setpoint);
    }

/**
 * Powers the shooter motors to run at varying speeds using the integrated velocity closed loop function [-18000, 18000]
 * @param setpoint Setpoint of the motors
 */
    public void setShooterNativeVeloctiy(double setpoint)
    {
        shootLeft.set(ControlMode.Velocity, setpoint);
        shootRight.set(ControlMode.Velocity, setpoint);
    }

    /*
    public void setShooterNativeVeloCalc(double setpoint)
    {
         shootLeft.set(ControlMode.Velocity, setpoint);
         shootRight.set(ControlMode.Velocity, setpoint);
    }
    */

/**
 * Calculates feedforward for the shooter. i have been told i should be using majority kF, and little kP. lol. will do next comp
 * @param setpoint Native velocity setpoint in the {@link #setShooterNativeVelocity} function
 * @return Feedforward that should be added when setting a setpoint
 * @see {@link #setShooterNativeVeloctiy(setpoint)}
 */
    public double shooterFeedForward(double setpoint)
    {
        return SHOOT.maxError * (Math.cos((Constants.kPi / 2) * (1+(setpoint / SHOOT.maxVelo))));
        //return ((SHOOT.maxError * setpoint) / SHOOT.maxVelo);
    }

    public double getShootLeftVelocity()
    {
        return shootLeft.getSelectedSensorVelocity();
    }

    public double getShootRightVelocity()
    {
        return shootRight.getSelectedSensorVelocity();
    }

    public void shootErrorTestVelocity(double speed, double time)
    {
        setShooterNativeVeloctiy(speed);
        if(time > 1)
        {
            shootErrorTest[0] = (shootErrorTest[0] + avgShootSpeed)/2;
            shootErrorTest[1] = (shootErrorTest[1] + avgShootSpeed)/2;
            
            shootErrorTest[2] = (shootErrorTest[2] + leftShootVelocity)/2;
            shootErrorTest[3] = (shootErrorTest[3] + rightShootVelocity)/2;

            shootErrorTest[4] = (shootErrorTest[4] + leftShootVelocity)/2;
            shootErrorTest[5] = (shootErrorTest[5] + rightShootVelocity)/2;
        }
        else
        {
            shootErrorTest[0] = 0;
            shootErrorTest[1] = speed;
            shootErrorTest[2] = 0;
            shootErrorTest[3] = 0;
            shootErrorTest[4] = speed;
            shootErrorTest[5] = speed;
        }
    }


    public double getError(int err)
    {
        return shootErrorTest[err];
    }

/**0 shootErrZero, 1 shootErrFull, 2-3 shootMotorErrZero, 4-5 shootMotorErrFull (left, right)*/
    private double[] shootErrorTest = new double[6];

    private static class InstanceHolder
    {
        private static final Shooter mInstance = new Shooter();
    } 
    
}
