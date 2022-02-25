// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.INTAKE;


/**
 * 
 */
public class Intake {
    //TODO add another intake motor
    private TalonSRX intakeLeft = new TalonSRX(INTAKE.intakeLeftCANID);
    private TalonSRX intakeRight = new TalonSRX(INTAKE.intakeRightCANID);
    private TalonSRX rollers = new TalonSRX(INTAKE.rollersCANID);

    //private boolean isOut = false;
    //private boolean move = false;

    private Intake()
    {
        intakeLeft.configFactoryDefault();
        intakeLeft.setNeutralMode(NeutralMode.Brake);
        intakeLeft.setInverted(INTAKE.leftFlipped);
        intakeLeft.enableVoltageCompensation(true);
        intakeLeft.configVoltageCompSaturation(INTAKE.voltComp);
        intakeLeft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        intakeRight.configFactoryDefault();
        intakeRight.setNeutralMode(NeutralMode.Brake);
        intakeRight.setInverted(!INTAKE.leftFlipped);
        intakeRight.enableVoltageCompensation(true);
        intakeRight.configVoltageCompSaturation(INTAKE.voltComp);
        intakeRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
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
        SmartDashboard.putNumber("magencLeft", intakeLeft.getSelectedSensorPosition());
        SmartDashboard.putNumber("magencRight", intakeRight.getSelectedSensorPosition());
    }

    public void setIntakePercent(double setpoint)
    {
        intakeRight.set(ControlMode.PercentOutput, setpoint);
        intakeLeft.set(ControlMode.PercentOutput, setpoint);
    }

    public void setRollersPercent(double setpoint)
    {
        SmartDashboard.putNumber("hbhbrgfhbgrhbilsrfg", setpoint);
        rollers.set(ControlMode.PercentOutput, setpoint);
    }

    public void setMagPosition(double setpoint)
    {
        intakeLeft.setSelectedSensorPosition(setpoint);
    }

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
    }   

    private static class InstanceHolder
    {
        private static final Intake mInstance = new Intake();
    } 
}
