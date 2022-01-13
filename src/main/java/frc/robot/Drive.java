// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DRIVE;
import frc.robot.Constants.TURN;


/** Add your docs here. */
public class Drive {
    private TalonFX topTurnLeft = new TalonFX(TURN.topTurnLeftCANID);
    private TalonFX topTurnRight = new TalonFX(TURN.topTurnRightCANID);
    private TalonFX bottomTurnLeft = new TalonFX(TURN.bottomTurnLeftCANID);
    private TalonFX bottomTurnRight = new TalonFX(TURN.bottomTurnRightCANID);

    private CANCoder topTurnLeftEncoder = new CANCoder(TURN.topTurnLeftCANCoderCANID);
    private CANCoder topTurnRightEncoder = new CANCoder(TURN.topTurnRightCANCoderCANID);
    private CANCoder bottomTurnLeftEncoder = new CANCoder(TURN.bottomTurnLeftCANCoderCANID);
    private CANCoder bottomTurnRightEncoder = new CANCoder(TURN.bottomTurnRightCANCoderCANID);

    private double offsetTopLeftCANCoder;
    private double offsetTopRightCANCoder;
    private double offsetBottomLeftCANCoder;
    private double offsetBottomRightCANCoder;

    private PIDController turningPID = new PIDController(TURN.turnKP, TURN.turnKI, TURN.turnKD);
    private PIDController drivePID = new PIDController(DRIVE.driveKP, DRIVE.driveKI, DRIVE.driveKD);
    private PIDController driveEther = new PIDController(DRIVE.driveKP, DRIVE.driveKI, DRIVE.driveKD);


    private AHRS navX = new AHRS();


    private Drive()
    {
        topTurnLeft.configFactoryDefault();
        bottomTurnLeft.configFactoryDefault();
        topTurnRight.configFactoryDefault();
        bottomTurnRight.configFactoryDefault();


        topTurnLeft.setNeutralMode(NeutralMode.Brake);
        topTurnRight.setNeutralMode(NeutralMode.Brake);
        bottomTurnLeft.setNeutralMode(NeutralMode.Brake);
        bottomTurnRight.setNeutralMode(NeutralMode.Brake);


        topTurnLeft.config_kP(0, TURN.turnKP);
        topTurnLeft.config_kI(0, TURN.turnKI);
        topTurnLeft.config_kD(0, TURN.turnKD);
        topTurnLeft.config_kF(0, TURN.turnKF);

        topTurnRight.config_kP(0, TURN.turnKP);
        topTurnRight.config_kI(0, TURN.turnKI);
        topTurnRight.config_kD(0, TURN.turnKD);
        topTurnRight.config_kF(0, TURN.turnKF);

        bottomTurnLeft.config_kP(0, TURN.turnKP);
        bottomTurnLeft.config_kI(0, TURN.turnKI);
        bottomTurnLeft.config_kD(0, TURN.turnKD);
        bottomTurnLeft.config_kF(0, TURN.turnKF);

        bottomTurnRight.config_kP(0, TURN.turnKP);
        bottomTurnRight.config_kI(0, TURN.turnKI);
        bottomTurnRight.config_kD(0, TURN.turnKD);
        bottomTurnRight.config_kF(0, TURN.turnKF);


        topTurnLeft.setInverted(false);
        topTurnRight.setInverted(false);
        bottomTurnLeft.setInverted(false);
        bottomTurnRight.setInverted(false);


        topTurnLeft.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_25Ms);
        topTurnLeft.configVelocityMeasurementWindow(16);

        topTurnRight.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_25Ms);
        topTurnRight.configVelocityMeasurementWindow(16);

        bottomTurnLeft.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_25Ms);
        bottomTurnLeft.configVelocityMeasurementWindow(16);

        bottomTurnRight.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_25Ms);
        bottomTurnRight.configVelocityMeasurementWindow(16);
   

        topTurnLeft.configVoltageCompSaturation(TURN.voltComp);
        topTurnLeft.enableVoltageCompensation(true);

        topTurnRight.configVoltageCompSaturation(TURN.voltComp);
        topTurnRight.enableVoltageCompensation(true);

        bottomTurnLeft.configVoltageCompSaturation(TURN.voltComp);
        bottomTurnLeft.enableVoltageCompensation(true);

        bottomTurnRight.configVoltageCompSaturation(TURN.voltComp);
        bottomTurnRight.enableVoltageCompensation(true);


        topTurnLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10);
        topTurnLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10);

        topTurnRight.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10);
        topTurnRight.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10);

        bottomTurnLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10);
        bottomTurnLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10);

        bottomTurnRight.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10);
        bottomTurnRight.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10);


        topTurnLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        topTurnRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        bottomTurnLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        bottomTurnRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);


        topTurnLeft.configFeedbackNotContinuous(true, 15);
        topTurnRight.configFeedbackNotContinuous(true, 15);
        bottomTurnLeft.configFeedbackNotContinuous(true, 15);
        bottomTurnRight.configFeedbackNotContinuous(true, 15);

        
        topTurnLeftEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        topTurnRightEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        bottomTurnLeftEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        bottomTurnRightEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);


        offsetTopLeftCANCoder = topTurnLeftEncoder.getAbsolutePosition() - TURN.topLeftOffset;//-107.050781;//78.75;
        offsetTopRightCANCoder = topTurnRightEncoder.getAbsolutePosition() - TURN.topRightOffset;//-67.3242187;//115.224;
        offsetBottomLeftCANCoder = bottomTurnLeftEncoder.getAbsolutePosition() - TURN.bottomLeftOffset;//-63.89648437; //117.0703125;//121.289;
        offsetBottomRightCANCoder = bottomTurnRightEncoder.getAbsolutePosition() - TURN.bottomRightOffset;//134.1210937;//320.361;
    }

    public static Drive getInstance()
    {
        return InstanceHolder.mInstance;
    }

    public void driveUpdate()
    {
        SmartDashboard.putNumber("topturnleft", topTurnLeft.getSelectedSensorPosition());
        SmartDashboard.putNumber("topturnright", topTurnRight.getSelectedSensorPosition());
        SmartDashboard.putNumber("bottomturnleft", bottomTurnLeft.getSelectedSensorPosition());
        SmartDashboard.putNumber("bottomturnright", bottomTurnRight.getSelectedSensorPosition());

        SmartDashboard.putNumber("encoderTopLeft", topTurnLeftEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("encoderTopRight", topTurnRightEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("encoderBotLeft", bottomTurnLeftEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("encoderBotRight", bottomTurnRightEncoder.getAbsolutePosition());
    }





    public void turnPercent(double setpoint)
    {
        topTurnLeft.set(ControlMode.PercentOutput, setpoint);
        topTurnRight.set(ControlMode.PercentOutput, setpoint);
        bottomTurnLeft.set(ControlMode.PercentOutput, setpoint);
        bottomTurnRight.set(ControlMode.PercentOutput, setpoint);
    }

    public void resetTurn()
    {
        topTurnLeft.setSelectedSensorPosition(0);
        topTurnRight.setSelectedSensorPosition(0);
        bottomTurnLeft.setSelectedSensorPosition(0);
        bottomTurnRight.setSelectedSensorPosition(0);
    }

    public void resetNavx()
    {
        navX.reset();
    }

    


    public double topTurnLeftCalculateNative(double setpoint)
    {
        return turningPID.calculate(topTurnLeft.getSelectedSensorPosition(), setpoint);
    }

    public double topTurnRightCalculateNative(double setpoint)
    {
        return turningPID.calculate(topTurnRight.getSelectedSensorPosition(), setpoint);
    }

    public double bottomTurnLeftCalculateNative(double setpoint)
    {
        return turningPID.calculate(bottomTurnLeft.getSelectedSensorPosition(), setpoint);
    }

    public double bottomTurnRightCalculateNative(double setpoint)
    {
        return turningPID.calculate(bottomTurnRight.getSelectedSensorPosition(), setpoint);
    }





    public void etherSwerve(double FWD, double STR, double RCW)
    {
        double yaw = navX.getYaw();
        double temp = (FWD * Math.cos(yaw)) + (STR* Math.sin(yaw));
        STR = (-FWD * Math.sin(yaw)) + (STR * Math.cos(yaw));
        FWD = temp;

        A = STR - RCW*(Constants.L/Constants.R);
        B = STR + RCW*(Constants.L/Constants.R);
        C = FWD - RCW*(Constants.W/Constants.R);
        D = FWD + RCW*(Constants.W/Constants.R);
        
        ws1 = Math.sqrt((Math.pow(B, 2)) + (Math.pow(C, 2)));      wa1 = Math.atan2(B,C)*180/Constants.kPi;
        ws2 = Math.sqrt((Math.pow(B, 2)) + (Math.pow(D, 2)));      wa2 = Math.atan2(B,D)*180/Constants.kPi;
        ws3 = Math.sqrt((Math.pow(A, 2)) + (Math.pow(D, 2)));      wa3 = Math.atan2(A,D)*180/Constants.kPi;
        ws4 = Math.sqrt((Math.pow(A, 2)) + (Math.pow(C, 2)));      wa4 = Math.atan2(A,C)*180/Constants.kPi; 

        max=ws1; if(ws2>max)max=ws2; if(ws3>max)max=ws3; if(ws4>max)max=ws4;
        if(max>1){ws1/=max; ws2/=max; ws3/=max; ws4/=max;}

        topTurnLeft.set(ControlMode.PercentOutput, topTurnLeftCalculateNative(MkUtil.degreesToNative(wa1, TURN.greerRatio)));
        topTurnRight.set(ControlMode.PercentOutput, topTurnRightCalculateNative(MkUtil.degreesToNative(wa2, TURN.greerRatio)));
        bottomTurnLeft.set(ControlMode.PercentOutput, bottomTurnLeftCalculateNative(MkUtil.degreesToNative(wa3, TURN.greerRatio)));
        bottomTurnRight.set(ControlMode.PercentOutput, bottomTurnRightCalculateNative(MkUtil.degreesToNative(wa4, TURN.greerRatio)));
    }


    public double ws1;
    public double ws2;
    public double ws3;
    public double ws4;

    public double wa1;
    public double wa2;
    public double wa3;
    public double wa4;

    public double A;
    public double B;
    public double C;
    public double D;

    public double max;

    private static class InstanceHolder
    {
        private static final Drive mInstance = new Drive();
    } 
}
