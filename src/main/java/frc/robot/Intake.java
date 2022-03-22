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
import frc.robot.Constants.INTAKE;


/**The Intake class contains everything related to the intake, including deployers and rollers*/
public class Intake {
    //// add another intake motor
    private TalonFX intake = new TalonFX(INTAKE.intakeCANID);
    //private TalonSRX intakeRight = new TalonSRX(INTAKE.intakeRightCANID);
    private TalonSRX rollers = new TalonSRX(INTAKE.rollersCANID);

    //private boolean isOut = false;
    //private boolean move = false;

    private Intake()
    {
        
        intake.configFactoryDefault();
        intake.setNeutralMode(NeutralMode.Brake);
        intake.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        intake.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_25Ms);
        intake.configVelocityMeasurementWindow(INTAKE.velocityMeasAmount);
        intake.enableVoltageCompensation(true);
        intake.configVoltageCompSaturation(INTAKE.voltComp);
        intake.config_kP(0, INTAKE.intakeKP);
        intake.config_kI(0, INTAKE.intakeKI);
        intake.config_kD(0, INTAKE.intakeKD);
        intake.config_kF(0, INTAKE.intakeKF);
        intake.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, INTAKE.statusOneMeas);
        intake.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, INTAKE.statusTwoMeas);

        

        //intakeRight.follow(intakeLeft);
        
        //intake.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        //intake.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

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

    public void updateIntake()
    {
        SmartDashboard.putNumber("position intake", intake.getSelectedSensorPosition());
       // SmartDashboard.putNumber("magencRight", intakeRight.getSelectedSensorPosition());
    }

    public void setIntakePercent(double setpoint)
    {
        intake.set(ControlMode.PercentOutput, setpoint);
      //  intakeLeft.set(ControlMode.PercentOutput, setpoint);
    }

    public void setRollersPercent(double setpoint)
    {
        //SmartDashboard.putNumber("Rollers", setpoint);
        rollers.set(ControlMode.PercentOutput, setpoint);
    }

    public void setIntakeEncoderPosition(double setpoint)
    {
        intake.setSelectedSensorPosition(setpoint);
    }

/**
 * Diy intake
 * @param button Toggle button
 */
/*
    public void bangBangIntake(boolean button) //TODO need to test to see if it works
    {
        if(intakeLeft.getSelectedSensorPosition() <= INTAKE.intakeInMaxError)
        {
            if(button)
            {
                setIntakePercent(INTAKE.intakeBangBangSpeed);
            }
            else
            {
                setIntakePercent(0);
            }
        }
        else if(intakeLeft.getSelectedSensorPosition() > INTAKE.intakeInMaxError && intakeLeft.getSelectedSensorPosition() < INTAKE.intakeRotationsNative - INTAKE.intakeOutThreshold)
        {
            if(button)
            {
                setIntakePercent(INTAKE.intakeBangBangSpeed);
            }
            else
            {
                setIntakePercent(-INTAKE.intakeBangBangSpeed);
            }
        }
        else if(intakeLeft.getSelectedSensorPosition() > INTAKE.intakeInMaxError && intakeLeft.getSelectedSensorPosition() >= INTAKE.intakeRotationsNative - INTAKE.intakeOutThreshold)
        {
            if(button)
            {
                setIntakePercent(0);
            }
            else
            {
                setIntakePercent(-INTAKE.intakeBangBangSpeed);
            }
        }
    }   */


    public void setIntakePosition(double setpoint)
    {
        intake.set(ControlMode.Position, setpoint);
    }

    private static class InstanceHolder
    {
        private static final Intake mInstance = new Intake();
    } 
}
