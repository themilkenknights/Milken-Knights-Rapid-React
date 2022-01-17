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

    private TalonFX topDriveLeft = new TalonFX(DRIVE.topDriveLeftCANID);    
    private TalonFX topDriveRight = new TalonFX(DRIVE.topDriveRightCANID);    
    private TalonFX bottomDriveLeft = new TalonFX(DRIVE.bottomDriveLeftCANID);
    private TalonFX bottomDriveRight = new TalonFX(DRIVE.bottomDriveRightCANID);   


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
    private PIDController driveTopLeftEther = new PIDController(DRIVE.driveKP, DRIVE.driveKI, DRIVE.driveKD);
    private PIDController driveTopRightEther = new PIDController(DRIVE.driveKP, DRIVE.driveKI, DRIVE.driveKD);
    private PIDController driveBotLeftEther = new PIDController(DRIVE.driveKP, DRIVE.driveKI, DRIVE.driveKD);
    private PIDController driveBotRightEther = new PIDController(DRIVE.driveKP, DRIVE.driveKI, DRIVE.driveKD);


    private AHRS navX = new AHRS();

    private double distance;

    private double leftTopPosNative, leftBottomPosNative, 
    rightTopPosNative, rightBottomPosNative,
    
    leftTopPosInch, leftBottomPosInch,
    rightTopPosInch, rightBottomPosInch,

    leftTopPosMeters, leftBottomPosMeters,
    rightTopPosMeters, rightBottomPosMeters,
    
    leftTopVelInch, leftBottomVelInch,
    rightTopVelInch, rightBottomVelInch,

    leftTopVelNative, leftBottomVelNative,
    rightTopVelNative, rightBottomVelNative,
    
    leftTopVelMeters, leftBottomVelMeters,
    rightTopVelMeters, rightBottomVelMeters,
    
    avgVelInches, avgDistInches,

    leftTopDeg, leftBottomDeg,
    rightTopDeg, rightBottomDeg,

    leftTopOutput, leftBottomOutput,
    rightTopOutput, rightBottomOutput;


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


        topTurnLeft.setInverted(true);
        topTurnRight.setInverted(true);
        bottomTurnLeft.setInverted(true);
        bottomTurnRight.setInverted(true);


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



        topTurnLeftEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        topTurnRightEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        bottomTurnLeftEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        bottomTurnRightEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);


        
        topTurnLeftEncoder.configSensorDirection(true);
        topTurnRightEncoder.configSensorDirection(true);
        bottomTurnLeftEncoder.configSensorDirection(true);
        bottomTurnRightEncoder.configSensorDirection(true);
/*
        offsetTopLeftCANCoder = ((topTurnLeftEncoder.getAbsolutePosition() + 360) % 360) - TURN.topLeftOffset;//-107.050781;//78.75;
        offsetTopRightCANCoder = ((topTurnRightEncoder.getAbsolutePosition() + 360) % 360) - TURN.topRightOffset;//-67.3242187;//115.224;
        offsetBottomLeftCANCoder = ((bottomTurnLeftEncoder.getAbsolutePosition() + 360) % 360) - TURN.bottomLeftOffset;//-63.89648437; //117.0703125;//121.289;
        offsetBottomRightCANCoder = ((bottomTurnRightEncoder.getAbsolutePosition() + 360) % 360) - TURN.bottomRightOffset;//134.1210937;//320.361;

        topTurnLeft.setSelectedSensorPosition(MkUtil.degreesToNative(offsetTopLeftCANCoder, TURN.greerRatio));
        topTurnRight.setSelectedSensorPosition(MkUtil.degreesToNative(offsetTopRightCANCoder, TURN.greerRatio));
        bottomTurnLeft.setSelectedSensorPosition(MkUtil.degreesToNative(offsetBottomLeftCANCoder, TURN.greerRatio));
        bottomTurnRight.setSelectedSensorPosition(MkUtil.degreesToNative(offsetBottomRightCANCoder, TURN.greerRatio));

        */
        topTurnLeftEncoder.configMagnetOffset(-TURN.topLeftOffset);
        topTurnRightEncoder.configMagnetOffset(-TURN.topRightOffset);
        bottomTurnLeftEncoder.configMagnetOffset(-TURN.bottomLeftOffset);
        bottomTurnRightEncoder.configMagnetOffset(-TURN.bottomRightOffset);

        topTurnLeft.setSelectedSensorPosition(MkUtil.degreesToNative(topTurnLeftEncoder.getAbsolutePosition(), TURN.greerRatio));
        topTurnRight.setSelectedSensorPosition(MkUtil.degreesToNative(topTurnRightEncoder.getAbsolutePosition(), TURN.greerRatio));
        bottomTurnLeft.setSelectedSensorPosition(MkUtil.degreesToNative(bottomTurnLeftEncoder.getAbsolutePosition(), TURN.greerRatio));
        bottomTurnRight.setSelectedSensorPosition(MkUtil.degreesToNative(bottomTurnRightEncoder.getAbsolutePosition(), TURN.greerRatio));


        topDriveLeft.configFactoryDefault();
        topDriveRight.configFactoryDefault();
        bottomDriveLeft.configFactoryDefault();
        bottomDriveRight.configFactoryDefault();


        topDriveLeft.setNeutralMode(NeutralMode.Brake);
        topDriveRight.setNeutralMode(NeutralMode.Brake);
        bottomDriveLeft.setNeutralMode(NeutralMode.Brake);
        bottomDriveRight.setNeutralMode(NeutralMode.Brake);


        topDriveLeft.setInverted(false);
        topDriveRight.setInverted(false);
        bottomDriveLeft.setInverted(false);
        bottomDriveRight.setInverted(false);


        topDriveLeft.configMotionSCurveStrength(6);
        topDriveRight.configMotionSCurveStrength(6);
        bottomDriveLeft.configMotionSCurveStrength(6);
        bottomDriveRight.configMotionSCurveStrength(6);


        topDriveLeft.config_kP(0, DRIVE.driveKP);
        topDriveLeft.config_kI(0, DRIVE.driveKI);
        topDriveLeft.config_kD(0, DRIVE.driveKD);
        topDriveLeft.config_kF(0, DRIVE.driveKF);

        topDriveRight.config_kP(0, DRIVE.driveKP);
        topDriveRight.config_kI(0, DRIVE.driveKI);
        topDriveRight.config_kD(0, DRIVE.driveKD);
        topDriveRight.config_kF(0, DRIVE.driveKF);

        bottomDriveLeft.config_kP(0, DRIVE.driveKP);
        bottomDriveLeft.config_kI(0, DRIVE.driveKI);
        bottomDriveLeft.config_kD(0, DRIVE.driveKD);
        bottomDriveLeft.config_kF(0, DRIVE.driveKF);

        bottomDriveRight.config_kP(0, DRIVE.driveKP);
        bottomDriveRight.config_kI(0, DRIVE.driveKI);
        bottomDriveRight.config_kD(0, DRIVE.driveKD);
        bottomDriveRight.config_kF(0, DRIVE.driveKF);


        topDriveLeft.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_25Ms);
        topDriveLeft.configVelocityMeasurementWindow(16);

        topDriveRight.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_25Ms);
        topDriveRight.configVelocityMeasurementWindow(16);

        bottomDriveLeft.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_25Ms);
        bottomDriveLeft.configVelocityMeasurementWindow(16);

        bottomDriveRight.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_25Ms);
        bottomDriveRight.configVelocityMeasurementWindow(16);


        topDriveLeft.configVoltageCompSaturation(DRIVE.voltComp);
        topDriveLeft.enableVoltageCompensation(true);
        
        topDriveRight.configVoltageCompSaturation(DRIVE.voltComp);
        topDriveRight.enableVoltageCompensation(true);

        bottomDriveLeft.configVoltageCompSaturation(DRIVE.voltComp);
        bottomDriveLeft.enableVoltageCompensation(true);

        bottomDriveRight.configVoltageCompSaturation(DRIVE.voltComp);
        bottomDriveRight.enableVoltageCompensation(true);


        topDriveLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10);
        topDriveLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10);

        topDriveRight.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10);
        topDriveRight.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10);

        bottomDriveLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10);
        bottomDriveLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10);

        bottomDriveRight.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10);
        bottomDriveRight.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10);


        topDriveLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        topDriveRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        bottomDriveLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        bottomDriveRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    }

    public static Drive getInstance()
    {
        return InstanceHolder.mInstance;
    }

    public void driveUpdate()
    {
        SmartDashboard.putNumber("topturnleft", bottomTurnLeft.getSelectedSensorPosition());
        SmartDashboard.putNumber("topturledeg", MkUtil.nativeToDegrees(bottomTurnLeft.getSelectedSensorPosition(), TURN.greerRatio));
        SmartDashboard.putNumber("topturnlefffff", MkUtil.degreesToNative(bottomTurnLeftEncoder.getAbsolutePosition(), TURN.greerRatio));

        SmartDashboard.putNumber("topturnright", bottomTurnRight.getSelectedSensorPosition());
        SmartDashboard.putNumber("topturrigdeg", MkUtil.nativeToDegrees(bottomTurnRight.getSelectedSensorPosition(), TURN.greerRatio));
        SmartDashboard.putNumber("topturnriiiii", MkUtil.degreesToNative(bottomTurnRightEncoder.getAbsolutePosition(), TURN.greerRatio));
       
       /* SmartDashboard.putNumber("bottomturnleft", bottomTurnLeft.getSelectedSensorPosition());
        SmartDashboard.putNumber("bottomturnright", bottomTurnRight.getSelectedSensorPosition());
*/
        SmartDashboard.putNumber("encoderTopLeft", topTurnLeftEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("encoderTopRight", topTurnRightEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("encoderBotLeft", bottomTurnLeftEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("encoderBotRight", bottomTurnRightEncoder.getAbsolutePosition());

        SmartDashboard.putNumber("currentDistance", currentDistance);

        SmartDashboard.putNumber("navx", navX.getYaw());

        leftTopVelNative = topDriveLeft.getSelectedSensorVelocity();
        rightTopVelNative = topDriveRight.getSelectedSensorVelocity();
        leftBottomVelNative = bottomDriveLeft.getSelectedSensorVelocity();
        rightBottomVelNative = bottomDriveRight.getSelectedSensorVelocity();

        leftTopPosNative = topDriveLeft.getSelectedSensorPosition();
        rightTopPosNative = topDriveRight.getSelectedSensorPosition();
        leftBottomPosNative = bottomDriveLeft.getSelectedSensorPosition();
        rightBottomPosNative = bottomDriveRight.getSelectedSensorPosition();

        leftTopVelInch = MkUtil.nativePer100MstoInchesPerSec(leftTopVelNative);
        rightTopVelInch = MkUtil.nativePer100MstoInchesPerSec(rightTopVelNative);
        leftBottomVelInch = MkUtil.nativePer100MstoInchesPerSec(leftBottomVelNative);
        rightBottomVelInch = MkUtil.nativePer100MstoInchesPerSec(rightBottomVelNative);

        leftTopPosInch = MkUtil.nativeToInches(leftTopPosNative);
        rightTopPosInch = MkUtil.nativeToInches(rightTopPosNative);
        leftBottomPosInch = MkUtil.nativeToInches(leftBottomPosNative);
        rightBottomPosInch = MkUtil.nativeToInches(rightBottomPosNative);
        
        leftTopVelMeters = MkUtil.nativePer100MsToMetersPerSec(leftTopVelNative);
        rightTopVelMeters = MkUtil.nativePer100MsToMetersPerSec(rightTopVelNative);
        leftBottomVelMeters = MkUtil.nativePer100MsToMetersPerSec(leftBottomVelNative);
        rightBottomVelMeters = MkUtil.nativePer100MsToMetersPerSec(rightBottomVelNative);

        leftTopPosMeters = MkUtil.nativeToMeters(leftTopPosNative);
        rightTopPosMeters = MkUtil.nativeToMeters(rightTopPosNative);
        leftBottomPosMeters = MkUtil.nativeToMeters(leftBottomPosNative);
        rightBottomPosMeters = MkUtil.nativeToMeters(rightBottomPosNative);

        avgDistInches = (leftTopPosInch + rightTopPosInch + leftBottomPosInch + rightBottomPosInch) /4.0;
        avgVelInches = (leftTopVelInch + rightTopVelInch + leftBottomVelInch + rightBottomVelInch) / 4.0;

        leftTopDeg = MkUtil.nativeToDegrees(topTurnLeft.getSelectedSensorPosition(), TURN.greerRatio);
        rightTopDeg = MkUtil.nativeToDegrees(topTurnRight.getSelectedSensorPosition(), TURN.greerRatio);
        leftBottomDeg = MkUtil.nativeToDegrees(bottomTurnLeft.getSelectedSensorPosition(), TURN.greerRatio);
        rightBottomDeg = MkUtil.nativeToDegrees(bottomTurnRight.getSelectedSensorPosition(), TURN.greerRatio);

    }





    public void turnPercent(double topleft, double topright, double botleft, double botright)
    {
        topTurnLeft.set(ControlMode.PercentOutput, topleft);
        topTurnRight.set(ControlMode.PercentOutput, topright);
        bottomTurnLeft.set(ControlMode.PercentOutput, botleft);
        bottomTurnRight.set(ControlMode.PercentOutput, botright);
    }

    public void turnCalcPercent(double topleft, double topright, double botleft, double botright)
    {
        topTurnLeft.set(ControlMode.PercentOutput, topTurnLeftCalculateNative(MkUtil.degreesToNative(topleft, TURN.greerRatio)));
        topTurnRight.set(ControlMode.PercentOutput, topTurnRightCalculateNative(MkUtil.degreesToNative(topright, TURN.greerRatio)));
        bottomTurnLeft.set(ControlMode.PercentOutput, bottomTurnLeftCalculateNative(MkUtil.degreesToNative(botleft, TURN.greerRatio)));
        bottomTurnRight.set(ControlMode.PercentOutput, bottomTurnRightCalculateNative(MkUtil.degreesToNative(botright, TURN.greerRatio)));
    }

    public void drivePercent(double topleft, double topright, double botleft, double botright)
    {
        topDriveLeft.set(ControlMode.PercentOutput, topleft);
        topDriveRight.set(ControlMode.PercentOutput, topright);
        bottomDriveLeft.set(ControlMode.PercentOutput, botleft);
        bottomDriveRight.set(ControlMode.PercentOutput, botright);
    }

    public void setPosTurn(double setpoint)
    {
        topTurnLeft.setSelectedSensorPosition(setpoint);
        topTurnRight.setSelectedSensorPosition(setpoint);
        bottomTurnLeft.setSelectedSensorPosition(setpoint);
        bottomTurnRight.setSelectedSensorPosition(setpoint);
    }

    public void resetTurn()
    {
        topTurnLeft.setSelectedSensorPosition(0);
        topTurnRight.setSelectedSensorPosition(0);
        bottomTurnLeft.setSelectedSensorPosition(0);
        bottomTurnRight.setSelectedSensorPosition(0);
    }

    public void resetDrive()
    {
        topDriveLeft.setSelectedSensorPosition(0);
        topDriveRight.setSelectedSensorPosition(0);
        bottomDriveLeft.setSelectedSensorPosition(0);
        bottomDriveRight.setSelectedSensorPosition(0);
    }

    public void encoderZero()
    {
        topTurnLeft.setSelectedSensorPosition(MkUtil.degreesToNative(topTurnLeftEncoder.getAbsolutePosition(), TURN.greerRatio));
        topTurnRight.setSelectedSensorPosition(MkUtil.degreesToNative(topTurnRightEncoder.getAbsolutePosition(), TURN.greerRatio));
        bottomTurnLeft.setSelectedSensorPosition(MkUtil.degreesToNative(bottomTurnLeftEncoder.getAbsolutePosition(), TURN.greerRatio));
        bottomTurnRight.setSelectedSensorPosition(MkUtil.degreesToNative(bottomTurnRightEncoder.getAbsolutePosition(), TURN.greerRatio));
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
        double temp = FWD * Math.cos(Math.toRadians(yaw)) + STR* Math.sin(Math.toRadians(yaw));
        STR = -FWD * Math.sin(Math.toRadians(yaw)) + STR * Math.cos(Math.toRadians(yaw));
        FWD = temp;

        double A = STR - RCW*(Constants.L/Constants.R);
        double B = STR + RCW*(Constants.L/Constants.R);
        double C = FWD - RCW*(Constants.W/Constants.R);
        double D = FWD + RCW*(Constants.W/Constants.R);
        
        double ws1 = Math.sqrt((Math.pow(B, 2)) + (Math.pow(C, 2)));      double wa1 = Math.atan2(B,C)*180/Constants.kPi;
        double ws2 = Math.sqrt((Math.pow(B, 2)) + (Math.pow(D, 2)));      double wa2 = Math.atan2(B,D)*180/Constants.kPi;
        double ws3 = Math.sqrt((Math.pow(A, 2)) + (Math.pow(D, 2)));      double wa3 = Math.atan2(A,D)*180/Constants.kPi;
        double ws4 = Math.sqrt((Math.pow(A, 2)) + (Math.pow(C, 2)));      double wa4 = Math.atan2(A,C)*180/Constants.kPi; 

        double max=ws1; if(ws2>max)max=ws2; if(ws3>max)max=ws3; if(ws4>max)max=ws4;
        if(max>1){ws1/=max; ws2/=max; ws3/=max; ws4/=max;}


        wa1 = MkUtil.setDirection(topTurnRight, wa1, driveTopRightEther);
        wa2 = MkUtil.setDirection(topTurnLeft, wa2, driveTopLeftEther);
        wa3 = MkUtil.setDirection(bottomTurnLeft, wa3, driveBotLeftEther);
        wa4 = MkUtil.setDirection(bottomTurnRight, wa4, driveBotRightEther);


        topTurnRight.set(ControlMode.PercentOutput, topTurnRightCalculateNative(MkUtil.degreesToNative(wa1, TURN.greerRatio)));
        topTurnLeft.set(ControlMode.PercentOutput, topTurnLeftCalculateNative(MkUtil.degreesToNative(wa2, TURN.greerRatio)));
        bottomTurnLeft.set(ControlMode.PercentOutput, bottomTurnLeftCalculateNative(MkUtil.degreesToNative(wa3, TURN.greerRatio)));
        bottomTurnRight.set(ControlMode.PercentOutput, bottomTurnRightCalculateNative(MkUtil.degreesToNative(wa4, TURN.greerRatio)));

        ws1 = MkUtil.isPositive(driveTopRightEther.getP(), ws1);
        ws2 = MkUtil.isPositive(driveTopLeftEther.getP(), ws2);
        ws3 = MkUtil.isPositive(driveBotLeftEther.getP(), ws3);
        ws4 = MkUtil.isPositive(driveBotRightEther.getP(), ws4);

        topDriveRight.set(ControlMode.PercentOutput, ws1);
        topDriveLeft.set(ControlMode.PercentOutput, ws2);
        bottomDriveLeft.set(ControlMode.PercentOutput, ws3);
        bottomDriveRight.set(ControlMode.PercentOutput, ws4);
    }


    
    public void swerveAutonomousEther(double FWD, double STR, double RCW)
    {
        double yaw = navX.getYaw();
        double temp = (FWD * Math.cos(yaw)) + (STR* Math.sin(yaw));
        STR = (-FWD * Math.sin(yaw)) + (STR * Math.cos(yaw));
        FWD = temp;


        double A = STR - RCW*(Constants.L/Constants.R);
        double B = STR + RCW*(Constants.L/Constants.R);
        double C = FWD - RCW*(Constants.W/Constants.R);
        double D = FWD + RCW*(Constants.W/Constants.R);
        

        double wa1 = Math.atan2(B,C)*180/Constants.kPi;
        double wa2 = Math.atan2(B,D)*180/Constants.kPi;
        double wa3 = Math.atan2(A,D)*180/Constants.kPi;
        double wa4 = Math.atan2(A,C)*180/Constants.kPi; 


       
        wa1 = MkUtil.setDirection(topTurnRight, wa1, driveTopRightEther);
        wa2 = MkUtil.setDirection(topTurnLeft, wa2, driveTopLeftEther);
        wa3 = MkUtil.setDirection(bottomTurnLeft, wa3, driveBotLeftEther);
        wa4 = MkUtil.setDirection(bottomTurnRight, wa4, driveBotRightEther);


        topTurnRight.set(ControlMode.PercentOutput, topTurnRightCalculateNative(MkUtil.degreesToNative(wa1, TURN.greerRatio)));
        topTurnLeft.set(ControlMode.PercentOutput, topTurnLeftCalculateNative(MkUtil.degreesToNative(wa2, TURN.greerRatio)));
        bottomTurnLeft.set(ControlMode.PercentOutput, bottomTurnLeftCalculateNative(MkUtil.degreesToNative(wa3, TURN.greerRatio)));
        bottomTurnRight.set(ControlMode.PercentOutput, bottomTurnRightCalculateNative(MkUtil.degreesToNative(wa4, TURN.greerRatio)));
    }








    public double currentDistance = 0;

    public double distanceA = 0;
    public double lengthB = 0;

    //let the math begin
    //in inches
    public double calculateCircleRadius(double distanceAA, double lengthBB)
    {
        return ((Math.pow(distanceAA, 2)/4) + Math.pow(lengthBB, 2)) * (1 / (2 * lengthBB));
    }
    public double calculateAngularVelocity(double distanceAA, double lengthBB)
    {
        double radius = calculateCircleRadius(distanceAA, lengthBB);
        return (DRIVE.maxInchesVelocity / radius);
    }
    public double calculateArcOfPath(double distanceAA, double lengthBB)
    {
        double radius = calculateCircleRadius(distanceAA, lengthBB);
        double theta = 2 * (Math.asin((distanceAA/(2 * radius))));
        return (theta / 360) * (2* (Constants.kPi * radius));
    }
    public double calculateAngleOfPath(double distanceAA, double lengthBB)
    {
        double radius = calculateCircleRadius(distanceAA, lengthBB);
        return 2 * (Math.asin((distanceAA/(2 * radius))));
    }


    public void autoTurnSet()
    {
        currentDistance = 0;
    }
    public void autoTurnUpdate(double totalDistance, double thetaTurn, double RCW)
    {
        currentDistance = 
            (MkUtil.nativeToInches(topDriveLeft.getSelectedSensorPosition()) +
            MkUtil.nativeToInches(topDriveRight.getSelectedSensorPosition()) +
            MkUtil.nativeToInches(bottomDriveLeft.getSelectedSensorPosition()) + 
            MkUtil.nativeToInches(bottomDriveRight.getSelectedSensorPosition())) / 4;

        double FWDauto = Math.sin(((currentDistance/totalDistance)*thetaTurn) * Constants.kPi / 180);
        double STRauto = Math.cos(((currentDistance/totalDistance)*thetaTurn) * Constants.kPi / 180);
        swerveAutonomousEther(FWDauto, STRauto, RCW);
/*
        topTurnLeft.set(ControlMode.PercentOutput, topTurnLeftCalculateNative(MkUtil.degreesToNative(((currentDistance/totalDistance)*thetaTurn), TURN.greerRatio)));
        topTurnRight.set(ControlMode.PercentOutput, topTurnRightCalculateNative(MkUtil.degreesToNative(((currentDistance/totalDistance)*thetaTurn), TURN.greerRatio)));
        bottomTurnLeft.set(ControlMode.PercentOutput, bottomTurnLeftCalculateNative(MkUtil.degreesToNative(((currentDistance/totalDistance)*thetaTurn), TURN.greerRatio)));
        bottomTurnRight.set(ControlMode.PercentOutput, bottomTurnRightCalculateNative(MkUtil.degreesToNative(((currentDistance/totalDistance)*thetaTurn), TURN.greerRatio)));
 */
    }

    public boolean autoTurnIsDone(double totalDistance)
    {
        return Math.abs(totalDistance - currentDistance) < 0.5 && Math.abs(avgVelInches) < 0.1;
    }



    public void setMagicStraight(double setpoint)
    {
        resetDrive();
        distance = setpoint;
        topDriveLeft.configMotionCruiseVelocity(DRIVE.magicVel);
        topDriveRight.configMotionCruiseVelocity(DRIVE.magicVel);
        bottomDriveLeft.configMotionCruiseVelocity(DRIVE.magicVel);
        bottomDriveRight.configMotionCruiseVelocity(DRIVE.magicVel);

        topDriveLeft.configMotionAcceleration(DRIVE.magicAccel);
        topDriveRight.configMotionAcceleration(DRIVE.magicAccel);
        bottomDriveLeft.configMotionAcceleration(DRIVE.magicAccel);
        bottomDriveRight.configMotionAcceleration(DRIVE.magicAccel);

        //zeroSensors();
    }

    public void updateMagicStraight()
    {
        topDriveLeft.set(ControlMode.MotionMagic, MkUtil.inchesToNative(distance));
        topDriveRight.set(ControlMode.MotionMagic, MkUtil.inchesToNative(distance));
        bottomDriveLeft.set(ControlMode.MotionMagic, MkUtil.inchesToNative(distance));
        bottomDriveRight.set(ControlMode.MotionMagic, MkUtil.inchesToNative(distance));

        leftTopOutput = MkUtil.inchesToNative(distance);
        rightTopOutput = MkUtil.inchesToNative(distance);
        leftBottomOutput = MkUtil.inchesToNative(distance);
        rightBottomOutput = MkUtil.inchesToNative(distance);

        SmartDashboard.putNumber("dist", distance);
        SmartDashboard.putNumber("leftout", leftTopOutput);
    }

    public boolean isMagicStraightDone()
    {
        double err = distance - avgDistInches;
        return Math.abs(err) < 0.5 && Math.abs(avgVelInches) < 0.1;
    }


    public boolean percentTurnDone()
    {
        return ((topTurnLeft.getMotorOutputPercent() +
               topTurnRight.getMotorOutputPercent() +
               bottomTurnLeft.getMotorOutputPercent() +
               bottomTurnRight.getMotorOutputPercent()) / 4) < 0.1;
    }





    private static class InstanceHolder
    {
        private static final Drive mInstance = new Drive();
    } 
}
