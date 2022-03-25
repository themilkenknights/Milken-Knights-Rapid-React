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


/**The Drive class contains all functions for anything related to DIY swerve drive (ether drive, DIY auto, etc)*/
public class Drive {

    public TalonFX topTurnLeft = new TalonFX(TURN.topTurnLeftCANID);
    public TalonFX topTurnRight = new TalonFX(TURN.topTurnRightCANID);
    public TalonFX bottomTurnLeft = new TalonFX(TURN.bottomTurnLeftCANID);
    public TalonFX bottomTurnRight = new TalonFX(TURN.bottomTurnRightCANID);
    
    public TalonFX topDriveLeft = new TalonFX(DRIVE.topDriveLeftCANID);    
    public TalonFX topDriveRight = new TalonFX(DRIVE.topDriveRightCANID);    
    public TalonFX bottomDriveLeft = new TalonFX(DRIVE.bottomDriveLeftCANID);
    public TalonFX bottomDriveRight = new TalonFX(DRIVE.bottomDriveRightCANID);   

    public CANCoder topTurnLeftEncoder = new CANCoder(TURN.topTurnLeftCANCoderCANID);
    public CANCoder topTurnRightEncoder = new CANCoder(TURN.topTurnRightCANCoderCANID);
    public CANCoder bottomTurnLeftEncoder = new CANCoder(TURN.bottomTurnLeftCANCoderCANID);
    public CANCoder bottomTurnRightEncoder = new CANCoder(TURN.bottomTurnRightCANCoderCANID);

    private PIDController turningPID = new PIDController(TURN.turnKP, TURN.turnKI, TURN.turnKD);
    private PIDController drivePID = new PIDController(DRIVE.driveKP, DRIVE.driveKI, DRIVE.driveKD);
    
    //bogus pid controllers, do nothing, still keep
    private PIDController driveTopLeftEther = new PIDController(DRIVE.driveKP, DRIVE.driveKI, DRIVE.driveKD);
    private PIDController driveTopRightEther = new PIDController(DRIVE.driveKP, DRIVE.driveKI, DRIVE.driveKD);
    private PIDController driveBotLeftEther = new PIDController(DRIVE.driveKP, DRIVE.driveKI, DRIVE.driveKD);
    private PIDController driveBotRightEther = new PIDController(DRIVE.driveKP, DRIVE.driveKI, DRIVE.driveKD);

    private double navXSHIT;

    public AHRS navX = new AHRS();

 /**Distance variable for driving in autonomous*/
    private double distance;

 /**Position of the driving motor in native units*/
    private double 
    leftTopPosNative, leftBottomPosNative, 
    rightTopPosNative, rightBottomPosNative;
    
 /**Position of the driving motor in inches*/
    private double
    leftTopPosInch, leftBottomPosInch,
    rightTopPosInch, rightBottomPosInch;

 /**Position of the driving motor in meters*/
    private double
    leftTopPosMeters, leftBottomPosMeters,
    rightTopPosMeters, rightBottomPosMeters;
    
 /**Velocity of the driving motor in inches*/
    private double
    leftTopVelInch, leftBottomVelInch,
    rightTopVelInch, rightBottomVelInch;

 /**Velocity of the driving motor in native units*/
    private double
    leftTopVelNative, leftBottomVelNative,
    rightTopVelNative, rightBottomVelNative;
    
 /**Velocity of the driving motor in meters*/
    private double
    leftTopVelMeters, leftBottomVelMeters,
    rightTopVelMeters, rightBottomVelMeters;

 /**Position of the turning motor in degrees*/
    private double
    leftTopDeg, leftBottomDeg,
    rightTopDeg, rightBottomDeg;

 /**Driving motor values for autonomous*/
    private double
    leftTopOutput, leftBottomOutput,
    rightTopOutput, rightBottomOutput;

 /**Average velocity of driving motors in inches*/
    private double avgVelInches;

 /**Average velocity of driving motors in native units*/
    private double avgVelNative;

 /**Average distance of driving motors in inches*/
    private double avgDistInches;

 /**Value for putting up highest velocity for drive motors*/
    private double topLeft = 0, topRight = 0, botLeft = 0, botRight = 0;

    private Drive()
    {
        topTurnLeft.configFactoryDefault();
        bottomTurnLeft.configFactoryDefault();
        topTurnRight.configFactoryDefault();
        bottomTurnRight.configFactoryDefault();


        topTurnLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        topTurnRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        bottomTurnLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        bottomTurnRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);



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
        topTurnLeft.configVelocityMeasurementWindow(TURN.velocityMeasAmount);

        topTurnRight.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_25Ms);
        topTurnRight.configVelocityMeasurementWindow(TURN.velocityMeasAmount);

        bottomTurnLeft.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_25Ms);
        bottomTurnLeft.configVelocityMeasurementWindow(TURN.velocityMeasAmount);

        bottomTurnRight.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_25Ms);
        bottomTurnRight.configVelocityMeasurementWindow(TURN.velocityMeasAmount);


        /*topTurnLeft.configSelectedFeedbackCoefficient(1.0 / 10.75);
        topTurnRight.configSelectedFeedbackCoefficient(1.0 / 10.75);
        bottomTurnLeft.configSelectedFeedbackCoefficient(1.0 / 10.75);
        bottomTurnRight.configSelectedFeedbackCoefficient(1.0 / 10.75);*/

        topTurnLeft.configVoltageCompSaturation(TURN.voltComp);
        topTurnLeft.enableVoltageCompensation(true);

        topTurnRight.configVoltageCompSaturation(TURN.voltComp);
        topTurnRight.enableVoltageCompensation(true);

        bottomTurnLeft.configVoltageCompSaturation(TURN.voltComp);
        bottomTurnLeft.enableVoltageCompensation(true);

        bottomTurnRight.configVoltageCompSaturation(TURN.voltComp);
        bottomTurnRight.enableVoltageCompensation(true);


        topTurnLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, TURN.statusOneMeas);
        topTurnLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, TURN.statusTwoMeas);

        topTurnRight.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, TURN.statusOneMeas);
        topTurnRight.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, TURN.statusTwoMeas);

        bottomTurnLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, TURN.statusOneMeas);
        bottomTurnLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, TURN.statusTwoMeas);

        bottomTurnRight.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, TURN.statusOneMeas);
        bottomTurnRight.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, TURN.statusTwoMeas);



        topTurnLeftEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        topTurnRightEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        bottomTurnLeftEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        bottomTurnRightEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);


        
        topTurnLeftEncoder.configSensorDirection(true);
        topTurnRightEncoder.configSensorDirection(true);
        bottomTurnLeftEncoder.configSensorDirection(true);
        bottomTurnRightEncoder.configSensorDirection(true);


/*      offsetTopLeftCANCoder = ((topTurnLeftEncoder.getAbsolutePosition() + 360) % 360) - TURN.topLeftOffset;//-107.050781;//78.75;
        offsetTopRightCANCoder = ((topTurnRightEncoder.getAbsolutePosition() + 360) % 360) - TURN.topRightOffset;//-67.3242187;//115.224;
        offsetBottomLeftCANCoder = ((bottomTurnLeftEncoder.getAbsolutePosition() + 360) % 360) - TURN.bottomLeftOffset;//-63.89648437; //117.0703125;//121.289;
        offsetBottomRightCANCoder = ((bottomTurnRightEncoder.getAbsolutePosition() + 360) % 360) - TURN.bottomRightOffset;//134.1210937;//320.361;

        topTurnLeft.setSelectedSensorPosition(MkUtil.degreesToNative(offsetTopLeftCANCoder, TURN.greerRatio));
        topTurnRight.setSelectedSensorPosition(MkUtil.degreesToNative(offsetTopRightCANCoder, TURN.greerRatio));
        bottomTurnLeft.setSelectedSensorPosition(MkUtil.degreesToNative(offsetBottomLeftCANCoder, TURN.greerRatio));
        bottomTurnRight.setSelectedSensorPosition(MkUtil.degreesToNative(offsetBottomRightCANCoder, TURN.greerRatio));*/

        topTurnLeftEncoder.configMagnetOffset(TURN.topLeftOffset);
        topTurnRightEncoder.configMagnetOffset(TURN.topRightOffset);
        bottomTurnLeftEncoder.configMagnetOffset(TURN.bottomLeftOffset);
        bottomTurnRightEncoder.configMagnetOffset(TURN.bottomRightOffset);

        topTurnLeft.setSelectedSensorPosition(MkUtil.degreesToNative(topTurnLeftEncoder.getAbsolutePosition(), TURN.greerRatio));
        topTurnRight.setSelectedSensorPosition(MkUtil.degreesToNative(topTurnRightEncoder.getAbsolutePosition(), TURN.greerRatio));
        bottomTurnLeft.setSelectedSensorPosition(MkUtil.degreesToNative(bottomTurnLeftEncoder.getAbsolutePosition(), TURN.greerRatio));
        bottomTurnRight.setSelectedSensorPosition(MkUtil.degreesToNative(bottomTurnRightEncoder.getAbsolutePosition(), TURN.greerRatio));


        /*      topTurnLeft.configMotionSCurveStrength(6);
        topTurnRight.configMotionSCurveStrength(6);
        bottomTurnLeft.configMotionSCurveStrength(6);
        bottomTurnRight.configMotionSCurveStrength(6);*/


        topDriveLeft.configFactoryDefault();
        topDriveRight.configFactoryDefault();
        bottomDriveLeft.configFactoryDefault();
        bottomDriveRight.configFactoryDefault();


        topDriveLeft.setNeutralMode(NeutralMode.Brake);
        topDriveRight.setNeutralMode(NeutralMode.Brake);
        bottomDriveLeft.setNeutralMode(NeutralMode.Brake);
        bottomDriveRight.setNeutralMode(NeutralMode.Brake);


        topDriveLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        topDriveRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        bottomDriveLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        bottomDriveRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);


        topDriveLeft.setInverted(false);
        topDriveRight.setInverted(false);
        bottomDriveLeft.setInverted(false);
        bottomDriveRight.setInverted(false);


        topDriveLeft.configMotionSCurveStrength(DRIVE.magicSCurve);
        topDriveRight.configMotionSCurveStrength(DRIVE.magicSCurve);
        bottomDriveLeft.configMotionSCurveStrength(DRIVE.magicSCurve);
        bottomDriveRight.configMotionSCurveStrength(DRIVE.magicSCurve);


        topDriveLeft.config_kP(0, DRIVE.driveKP);
        topDriveLeft.config_kI(0, DRIVE.driveKI);
        topDriveLeft.config_kD(0, DRIVE.driveKD);
        topDriveLeft.config_kF(0, DRIVE.driveTopLeftKF);

        topDriveRight.config_kP(0, DRIVE.driveKP);
        topDriveRight.config_kI(0, DRIVE.driveKI);
        topDriveRight.config_kD(0, DRIVE.driveKD);
        topDriveRight.config_kF(0, DRIVE.driveTopRightKF);

        bottomDriveLeft.config_kP(0, DRIVE.driveKP);
        bottomDriveLeft.config_kI(0, DRIVE.driveKI);
        bottomDriveLeft.config_kD(0, DRIVE.driveKD);
        bottomDriveLeft.config_kF(0, DRIVE.driveBottomLeftKF);

        bottomDriveRight.config_kP(0, DRIVE.driveKP);
        bottomDriveRight.config_kI(0, DRIVE.driveKI);
        bottomDriveRight.config_kD(0, DRIVE.driveKD);
        bottomDriveRight.config_kF(0, DRIVE.driveBottomRightKF);


        topDriveLeft.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_25Ms);
        topDriveLeft.configVelocityMeasurementWindow(DRIVE.velocityMeasAmount);

        topDriveRight.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_25Ms);
        topDriveRight.configVelocityMeasurementWindow(DRIVE.velocityMeasAmount);

        bottomDriveLeft.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_25Ms);
        bottomDriveLeft.configVelocityMeasurementWindow(DRIVE.velocityMeasAmount);

        bottomDriveRight.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_25Ms);
        bottomDriveRight.configVelocityMeasurementWindow(DRIVE.velocityMeasAmount);


   /*   topDriveLeft.configSelectedFeedbackCoefficient(1.0 / 10.75);
        topDriveRight.configSelectedFeedbackCoefficient(1.0 / 10.75);
        bottomDriveLeft.configSelectedFeedbackCoefficient(1.0 / 10.75);
        bottomDriveRight.configSelectedFeedbackCoefficient(1.0 / 10.75);*/


        topDriveLeft.configVoltageCompSaturation(DRIVE.voltComp);
        topDriveLeft.enableVoltageCompensation(true);
        
        topDriveRight.configVoltageCompSaturation(DRIVE.voltComp);
        topDriveRight.enableVoltageCompensation(true);

        bottomDriveLeft.configVoltageCompSaturation(DRIVE.voltComp);
        bottomDriveLeft.enableVoltageCompensation(true);

        bottomDriveRight.configVoltageCompSaturation(DRIVE.voltComp);
        bottomDriveRight.enableVoltageCompensation(true);


        topDriveLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, DRIVE.statusOneMeas);
        topDriveLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, DRIVE.statusTwoMeas);

        topDriveRight.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, DRIVE.statusOneMeas);
        topDriveRight.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, DRIVE.statusTwoMeas);

        bottomDriveLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, DRIVE.statusOneMeas);
        bottomDriveLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, DRIVE.statusTwoMeas);

        bottomDriveRight.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, DRIVE.statusOneMeas);
        bottomDriveRight.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, DRIVE.statusTwoMeas);
    }

    public static Drive getInstance()
    {
        return InstanceHolder.mInstance;
    }

    public void driveUpdate()
    {
        //updateDriveDriveVelocity();
        //updateDriveTurn();
        //updateDriveTurnEncoder();
      
      
        //  SmartDashboard.putNumber("currentDistance", currentDistance);

        SmartDashboard.putNumber("navx", getNavx());
        //updateNavxCalibration();
       // SmartDashboard.putNumber("status", topTurnLeft.getStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0));
        
       // SmartDashboard.putNumber("top left vel", topTurnLeft.getMotorOutputPercent());

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
        avgVelNative = (leftTopVelNative + rightTopVelNative + leftBottomVelNative + rightBottomVelNative) / 4.0;

        leftTopDeg = MkUtil.nativeToDegrees(topTurnLeft.getSelectedSensorPosition(), TURN.greerRatio);
        rightTopDeg = MkUtil.nativeToDegrees(topTurnRight.getSelectedSensorPosition(), TURN.greerRatio);
        leftBottomDeg = MkUtil.nativeToDegrees(bottomTurnLeft.getSelectedSensorPosition(), TURN.greerRatio);
        rightBottomDeg = MkUtil.nativeToDegrees(bottomTurnRight.getSelectedSensorPosition(), TURN.greerRatio);

    }

 
    
    public void updateDriveDriveVelocity()
    {
       // SmartDashboard.putNumber("topleftvelocity", topLeft);
        if(topDriveLeft.getSelectedSensorVelocity() > topLeft)
        {
            topLeft = topDriveLeft.getSelectedSensorVelocity();
        }
      //  SmartDashboard.putNumber("toprightvelocity", topRight);
        if(topDriveRight.getSelectedSensorVelocity() > topRight)
        {
            topRight = topDriveRight.getSelectedSensorVelocity();
        }
      //  SmartDashboard.putNumber("bottomleftvelocity", botLeft);
        if(bottomDriveLeft.getSelectedSensorVelocity() > botLeft)
        {
            botLeft = bottomDriveLeft.getSelectedSensorVelocity();
        }
       // SmartDashboard.putNumber("bottomrightvelocity", botRight);
        if(bottomDriveRight.getSelectedSensorVelocity() > botRight)
        {
            botRight = bottomDriveRight.getSelectedSensorVelocity();
        }
    }

    public void updateDriveDriveRaw()
    {
   //     SmartDashboard.putNumber("topleftvelocity", topDriveLeft.getSelectedSensorVelocity());
   //     SmartDashboard.putNumber("toprightvelocity", topDriveRight.getSelectedSensorVelocity());
   //     SmartDashboard.putNumber("bottomleftvelocity", bottomDriveLeft.getSelectedSensorVelocity());
  //      SmartDashboard.putNumber("bottomrightvelocity", bottomDriveRight.getSelectedSensorVelocity());
    }

    public void updateDriveTurn()
    {
        //SmartDashboard.putNumber("topturnleft", bottomTurnLeft.getSelectedSensorPosition());
        //SmartDashboard.putNumber("topturledeg", MkUtil.nativeToDegrees(bottomTurnLeft.getSelectedSensorPosition(), TURN.greerRatio));
      //  SmartDashboard.putNumber("topturnlefffff", MkUtil.degreesToNative(bottomTurnLeftEncoder.getAbsolutePosition(), TURN.greerRatio));

    //    SmartDashboard.putNumber("topturnright", bottomTurnRight.getSelectedSensorPosition());
     //   SmartDashboard.putNumber("topturrigdeg", MkUtil.nativeToDegrees(bottomTurnRight.getSelectedSensorPosition(), TURN.greerRatio));
     //   SmartDashboard.putNumber("topturnriiiii", MkUtil.degreesToNative(bottomTurnRightEncoder.getAbsolutePosition(), TURN.greerRatio));
       
      //  SmartDashboard.putNumber("bottomturnleft", bottomTurnLeft.getSelectedSensorPosition());
      //  SmartDashboard.putNumber("bottomturnright", bottomTurnRight.getSelectedSensorPosition());
    }

    public void updateDriveTurnEncoder()
    {
   //     SmartDashboard.putNumber("encoderTopLeft", topTurnLeftEncoder.getAbsolutePosition());
  //      SmartDashboard.putNumber("encoderTopRight", topTurnRightEncoder.getAbsolutePosition());
   //     SmartDashboard.putNumber("encoderBotLeft", bottomTurnLeftEncoder.getAbsolutePosition());
   //     SmartDashboard.putNumber("encoderBotRight", bottomTurnRightEncoder.getAbsolutePosition());
    }

    public void updateNavxCalibration()
    {
        SmartDashboard.putBoolean("Navx Yaw", navX.isBoardlevelYawResetEnabled());
        SmartDashboard.putBoolean("Navx Calibrating", navX.isCalibrating());
        SmartDashboard.putBoolean("Navx calibrated", navX.isMagnetometerCalibrated());
    }



    public void turnPercent(double topleft, double topright, double botleft, double botright)
    {
        topTurnLeft.set(ControlMode.PercentOutput, topleft);
        topTurnRight.set(ControlMode.PercentOutput, topright);
        bottomTurnLeft.set(ControlMode.PercentOutput, botleft);
        bottomTurnRight.set(ControlMode.PercentOutput, botright);
    }

 /**Powers all angular motors to turn to specific angles (use degrees)*/
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

 /**Powers all drive motors at varying speeds [velocity in native units]*/
    public void driveVelocity(double topleft, double topright, double botleft, double botright)
    {
        topDriveLeft.set(ControlMode.Velocity, topleft + ((6* topleft * 1023 * DRIVE.greerRatio)/DRIVE.topLeftMaxNativeVelocity));
        topDriveRight.set(ControlMode.Velocity, topright + ((7*topright * 1023* DRIVE.greerRatio)/DRIVE.topRightMaxNativeVelocity));
        bottomDriveLeft.set(ControlMode.Velocity, botleft + ((6*botleft * 1023* DRIVE.greerRatio)/DRIVE.bottomLeftMaxNativeVelocity));
        bottomDriveRight.set(ControlMode.Velocity, botright + ((6*botright * 1023* DRIVE.greerRatio)/DRIVE.bottomRightMaxNativeVelocity));
  /*
        topDriveLeft.set(ControlMode.PercentOutput, topDriveLeftCalculateNative(topleft));
        topDriveRight.set(ControlMode.PercentOutput, topDriveRightCalculateNative(topright));
        bottomDriveLeft.set(ControlMode.PercentOutput, bottomDriveLeftCalculateNative(botleft));
        bottomDriveRight.set(ControlMode.PercentOutput, bottomDriveRightCalculateNative(botright));
   */
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
    
 /**Sets all angular motors' integrated encoder's positions to their respective CANCoder's position*/
    public void encoderZero()
    {
        topTurnLeft.setSelectedSensorPosition(MkUtil.degreesToNative(topTurnLeftEncoder.getAbsolutePosition(), TURN.greerRatio));
        topTurnRight.setSelectedSensorPosition(MkUtil.degreesToNative(topTurnRightEncoder.getAbsolutePosition(), TURN.greerRatio));
        bottomTurnLeft.setSelectedSensorPosition(MkUtil.degreesToNative(bottomTurnLeftEncoder.getAbsolutePosition(), TURN.greerRatio));
        bottomTurnRight.setSelectedSensorPosition(MkUtil.degreesToNative(bottomTurnRightEncoder.getAbsolutePosition(), TURN.greerRatio));
    }
    //TODO test to see if solves problem of continuous bullshit rotaty thingy (zoom zoom when at 0 continuous thing)
    
 /**Powers all angular motors to turn to 0 based on their respective CANCoder's position (hopefully)*/
    public void encoderMoveToZero()
    {   
        topTurnLeft.set(ControlMode.PercentOutput,  turningPID.calculate(MkUtil.degreesToNative(topTurnLeftEncoder.getAbsolutePosition(), TURN.greerRatio),0));
        topTurnRight.set(ControlMode.PercentOutput,  turningPID.calculate(MkUtil.degreesToNative(topTurnRightEncoder.getAbsolutePosition(), TURN.greerRatio),0));
        bottomTurnLeft.set(ControlMode.PercentOutput,  turningPID.calculate(MkUtil.degreesToNative(bottomTurnLeftEncoder.getAbsolutePosition(), TURN.greerRatio),0));
        bottomTurnRight.set(ControlMode.PercentOutput,  turningPID.calculate(MkUtil.degreesToNative(bottomTurnRightEncoder.getAbsolutePosition(), TURN.greerRatio),0));
   
    }

    public void resetNavx()
    {
        navX.zeroYaw();
    }

    /**
     * Gets navx yaw (minus 180 because of navx placement)
     * @return Navx yaw
     */
    public double getNavx()
    {
        return 180-navX.getYaw();
    }

    /**
     * Gets the top left angular motor's voltage
     * @return The applied voltage to the motor in volts. 
     */
    public double getTurnVolt()
    {
        return topTurnLeft.getMotorOutputVoltage();
    }

    /**
     * Gets top left drive motor's voltage
     * @return The applied voltage to the motor in volts. 
     */
    public double getDriveVolt()
    {
        return topDriveLeft.getMotorOutputVoltage();
    }

    /**
     * Takes a setpoint, and based on where the motor is, a PID controller calculates how fast the motor should move in order to reach said setpoint
     * @param setpoint Setpoint for the top left angular motor
     * @return A speed for the motor
     */
    public double topTurnLeftCalculateNative(double setpoint)
    {
        return turningPID.calculate(topTurnLeft.getSelectedSensorPosition(), setpoint);
    }

    /**
     * Takes a setpoint, and based on where the motor is, a PID controller calculates how fast the motor should move in order to reach said setpoint
     * @param setpoint Setpoint for the top right angular motor
     * @return A speed for the motor
     */
    public double topTurnRightCalculateNative(double setpoint)
    {
        return turningPID.calculate(topTurnRight.getSelectedSensorPosition(), setpoint);
    }

    /**
     * Takes a setpoint, and based on where the motor is, a PID controller calculates how fast the motor should move in order to reach said setpoint
     * @param setpoint Setpoint for the bottom left angular motor
     * @return A speed for the motor
     */
    public double bottomTurnLeftCalculateNative(double setpoint)
    {
        return turningPID.calculate(bottomTurnLeft.getSelectedSensorPosition(), setpoint);
    }

    /**
     * Takes a setpoint, and based on where the motor is, a PID controller calculates how fast the motor should move in order to reach said setpoint
     * @param setpoint Setpoint for the bottom right angular motor
     * @return A speed for the motor
     */
    public double bottomTurnRightCalculateNative(double setpoint)
    {
        return turningPID.calculate(bottomTurnRight.getSelectedSensorPosition(), setpoint);
    }








    /**
     * Takes a setpoint, and based on where the motor is, a PID controller calculates how fast the motor should move in order to reach said setpoint
     * @param setpoint Setpoint for the top left angular motor
     * @return A speed for the motor
     */
    public double topDriveLeftCalculateNative(double setpoint)
    {
        return drivePID.calculate(topDriveLeft.getSelectedSensorVelocity(), setpoint + ((setpoint * 2048 * DRIVE.greerRatio)/DRIVE.topLeftMaxNativeVelocity));
    }

    /**
     * Takes a setpoint, and based on where the motor is, a PID controller calculates how fast the motor should move in order to reach said setpoint
     * @param setpoint Setpoint for the top right angular motor
     * @return A speed for the motor
     */
    public double topDriveRightCalculateNative(double setpoint)
    {
        return drivePID.calculate(topDriveRight.getSelectedSensorVelocity(), setpoint + ((setpoint * 2048 * DRIVE.greerRatio)/DRIVE.topRightMaxNativeVelocity));
    }

    /**
     * Takes a setpoint, and based on where the motor is, a PID controller calculates how fast the motor should move in order to reach said setpoint
     * @param setpoint Setpoint for the bottom left angular motor
     * @return A speed for the motor
     */
    public double bottomDriveLeftCalculateNative(double setpoint)
    {
        return drivePID.calculate(bottomDriveLeft.getSelectedSensorVelocity(), setpoint + ((setpoint * 2048 * DRIVE.greerRatio)/DRIVE.bottomLeftMaxNativeVelocity));
    }

    /**
     * Takes a setpoint, and based on where the motor is, a PID controller calculates how fast the motor should move in order to reach said setpoint
     * @param setpoint Setpoint for the bottom right angular motor
     * @return A speed for the motor
     */
    public double bottomDriveRightCalculateNative(double setpoint)
    {
        return drivePID.calculate(bottomDriveRight.getSelectedSensorVelocity(), setpoint + ((setpoint * 2048 * DRIVE.greerRatio)/DRIVE.bottomRightMaxNativeVelocity));
    }













    /**
     * See <a href="https://www.chiefdelphi.com/t/paper-4-wheel-independent-drive-independent-steering-swerve/107383">this thread</a>
     * for more information 
     * <p>
     * Note - this function uses 180 minus yaw due to the positioning of our navX.
     * @param FWD Forward axis of controller
     * @param STR Strafe axis of controller
     * @param RCW Rotational axis of controller
     * @author Ether (?)
     * @see {@link #getNavx()}
     */
    public void etherSwerve(double FWD, double STR, double RCW)
    {
        double yaw = getNavx();
        double temp = FWD * Math.cos(Math.toRadians(yaw)) + STR * Math.sin(Math.toRadians(yaw));
        STR = -FWD * Math.sin(Math.toRadians(yaw)) + STR * Math.cos(Math.toRadians(yaw));
        FWD = temp;

        //SmartDashboard.putNumber("frd", FWD);
        //SmartDashboard.putNumber("str", STR);

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

        topTurnRight.set(ControlMode.Position, MkUtil.degreesToNative(wa1, TURN.greerRatio)); // ControlMode.PercentOutput, topTurnRightCalculateNative(MkUtil.degreesToNative(wa1, TURN.greerRatio)));
        topTurnLeft.set(ControlMode.Position, MkUtil.degreesToNative(wa2, TURN.greerRatio)); //ControlMode.PercentOutput, topTurnLeftCalculateNative(MkUtil.degreesToNative(wa2, TURN.greerRatio)));
        bottomTurnLeft.set(ControlMode.Position, MkUtil.degreesToNative(wa3, TURN.greerRatio)); //ControlMode.PercentOutput, bottomTurnLeftCalculateNative(MkUtil.degreesToNative(wa3, TURN.greerRatio)));
        bottomTurnRight.set(ControlMode.Position, MkUtil.degreesToNative(wa4, TURN.greerRatio)); //ControlMode.PercentOutput, bottomTurnRightCalculateNative(MkUtil.degreesToNative(wa4, TURN.greerRatio)));

        ws1 = MkUtil.isPositive(driveTopRightEther.getP(), ws1);
        ws2 = MkUtil.isPositive(driveTopLeftEther.getP(), ws2);
        ws3 = MkUtil.isPositive(driveBotLeftEther.getP(), ws3);
        ws4 = MkUtil.isPositive(driveBotRightEther.getP(), ws4);

        //SmartDashboard.putNumber("wa1",wa1);
        //SmartDashboard.putNumber("wa2",wa2);
        //SmartDashboard.putNumber("wa3",wa3);
        //SmartDashboard.putNumber("wa4",wa4);
/*
        topDriveRight.set(ControlMode.PercentOutput, ws1);
        topDriveLeft.set(ControlMode.PercentOutput, ws2);
        bottomDriveLeft.set(ControlMode.PercentOutput, ws3);
        bottomDriveRight.set(ControlMode.PercentOutput, ws4);
    */

        driveVelocity(ws2 * 21600, ws1 * 21600, ws3 * 21600, ws4 * 21600);


    }




//TODO if this works credit them
    double hP = 0.001, hI = 0.0001, hD = hP * 0.1;
    double hIntegral, hDerivative, hPreviousError, hError;

    //programming done right
    public double headerStraighter(double hSetpoint)
    {
        hError = hSetpoint -  getNavx();// Error = Target - Actual
        hIntegral += (hError*.02); // Integral is increased by the error*time (which is .02 seconds using normal IterativeRobot)
        hDerivative = (hError - hPreviousError) / .02;
        return hP*hError + hI*hIntegral + hD*hDerivative;
        
    }

    public void updateHeaderFix(double hAngle)
    {
        SmartDashboard.putNumber("header return", headerStraighter(hAngle));
    }









    /**
     * same as {@link #etherSwerve}, but it only powers the angular motors
     * <p>
     * Note - this function uses 180 minus yaw due to the positioning of our navX.
     * @param FWD forward input
     * @param STR strafe input
     * @param RCW rotational input
     * @author Ether (?)
     * @see {@link #etherSwerve(FWD, STR, RCW)}
     * @see {@link #getNavx()}
     */
    public void swerveAutonomousEther(double FWD, double STR, double RCW)
    {
        double yaw = getNavx();
        double temp = FWD * Math.cos(Math.toRadians(yaw)) + STR * Math.sin(Math.toRadians(yaw));
        STR = -FWD * Math.sin(Math.toRadians(yaw)) + STR * Math.cos(Math.toRadians(yaw));
        FWD = temp;


        double A = STR - RCW*(Constants.L/Constants.R);
        double B = STR + RCW*(Constants.L/Constants.R);
        double C = FWD - RCW*(Constants.W/Constants.R);
        double D = FWD + RCW*(Constants.W/Constants.R);
        

        double wa1 = Math.atan2(B,C)*180/Constants.kPi;
        double wa2 = Math.atan2(B,D)*180/Constants.kPi;
        double wa3 = Math.atan2(A,D)*180/Constants.kPi;
        double wa4 = Math.atan2(A,C)*180/Constants.kPi; 

        double ws1 = Math.sqrt((Math.pow(B, 2)) + (Math.pow(C, 2)));      
        double ws2 = Math.sqrt((Math.pow(B, 2)) + (Math.pow(D, 2)));     
        double ws3 = Math.sqrt((Math.pow(A, 2)) + (Math.pow(D, 2)));    
        double ws4 = Math.sqrt((Math.pow(A, 2)) + (Math.pow(C, 2)));   

        double max=ws1; if(ws2>max)max=ws2; if(ws3>max)max=ws3; if(ws4>max)max=ws4;
        if(max>1){ws1/=max; ws2/=max; ws3/=max; ws4/=max;}
       
        wa1 = MkUtil.setDirection(topTurnRight, wa1, driveTopRightEther);
        wa2 = MkUtil.setDirection(topTurnLeft, wa2, driveTopLeftEther);
        wa3 = MkUtil.setDirection(bottomTurnLeft, wa3, driveBotLeftEther);
        wa4 = MkUtil.setDirection(bottomTurnRight, wa4, driveBotRightEther);

        


        topTurnRight.set(ControlMode.Position, MkUtil.degreesToNative(wa1, TURN.greerRatio)); // ControlMode.PercentOutput, topTurnRightCalculateNative(MkUtil.degreesToNative(wa1, TURN.greerRatio)));
        topTurnLeft.set(ControlMode.Position, MkUtil.degreesToNative(wa2, TURN.greerRatio)); //ControlMode.PercentOutput, topTurnLeftCalculateNative(MkUtil.degreesToNative(wa2, TURN.greerRatio)));
        bottomTurnLeft.set(ControlMode.Position, MkUtil.degreesToNative(wa3, TURN.greerRatio)); //ControlMode.PercentOutput, bottomTurnLeftCalculateNative(MkUtil.degreesToNative(wa3, TURN.greerRatio)));
        bottomTurnRight.set(ControlMode.Position, MkUtil.degreesToNative(wa4, TURN.greerRatio)); //ControlMode.PercentOutput, bottomTurnRightCalculateNative(MkUtil.degreesToNative(wa4, TURN.greerRatio)));


        ws1 = MkUtil.isPositive(driveTopRightEther.getP(), ws1)/8;
        ws2 = MkUtil.isPositive(driveTopLeftEther.getP(), ws2)/8;
        ws3 = MkUtil.isPositive(driveBotLeftEther.getP(), ws3)/8;
        ws4 = MkUtil.isPositive(driveBotRightEther.getP(), ws4)/8;

        driveVelocity(ws2 * 21600, ws1 * 21600, ws3 * 21600, ws4 * 21600);
    }







//for autonomous, so i dont have to make new variables
    public double currentDistance = 0;
    public double turnDistance = 0;
    public double distanceA = 0;
    public double lengthB = 0;
    public double FWDauto = 0;
    public double STRauto = 0;
   //public double RCWtemp = 0;

    /**
    Calculates a curved autonomous path's radius by using the distance between the starting and ending point and the distance between the middle of the path and the height of the angular path
    <pre>
    .
                         
                          E
                      ~~~~~~~~~
                   ~~     +     ~~     
  /o----o\       ~~       + B     ~~       /o----o\
  |  (F) |  (2) ~~        +        ~~ (1)  |  (F) |
  \o----o/      ==========A==========      \o----o/ 
                \         |         /    
                 \        |        /
                  \       |       /
                   \      |D     / 
                    \     |     / 
                     \  __|__  / 
                      \/  C  \/
                       \  |  /
                        \ | /
                         \|/
            
    A = distanceA / =
    B = lengthB / +
    C = angle
    D = radius / |
    E = circumference / ~
    F = robot
    1 = starting position
    2 = ending position
    (diagram above isnt a hot air balloon fyi)
    </pre>

    * @param distanceA
    * @param lengthB
    * @return Radius of the path
    */
    public double calculateCircleRadius(double distanceA, double lengthB)
    {
        return ((Math.pow(distanceA, 2)/4) + Math.pow(lengthB, 2)) * (1 / (2 * lengthB));
    }
    
    /*public double calculateAngularVelocity(double distanceA, double lengthB)
    {
        double radius = calculateCircleRadius(distanceA, lengthB);
        return (DRIVE.maxInchesVelocity / radius);
    }*/
    /**
    Calculates a curved autonomous path's circumference/length by using the distance between the starting and ending point and the distance between the middle of the linear path and the max height of the angular path
    <pre>
    .
                         
                          E
                      ~~~~~~~~~
                   ~~     +     ~~     
  /o----o\       ~~       + B     ~~       /o----o\
  |  (F) |  (2) ~~        +        ~~ (1)  |  (F) |
  \o----o/      ==========A==========      \o----o/ 
                \         |         /    
                 \        |        /
                  \       |       /
                   \      |D     / 
                    \     |     / 
                     \  __|__  / 
                      \/  C  \/
                       \  |  /
                        \ | /
                         \|/
            
    A = distanceA / =
    B = lengthB / +
    C = angle
    D = radius / |
    E = circumference / ~
    F = robot
    1 = starting position
    2 = ending position
    (diagram above isnt a hot air balloon fyi)
    </pre>
    * @param distanceA
    * @param lengthB
    * @return Circumference of the linear path / distance of curved path
    * @see {@link #calculateCircleRadius(distanceA, lengthB)}
    */
    public double calculateArcOfPath(double distanceA, double lengthB)
    {
        double radius = calculateCircleRadius(distanceA, lengthB);
        double theta = 2 * (Math.toDegrees((Math.asin((distanceA/(2 * radius))))));
        return (theta / 360) * (2 * (Constants.kPi * radius));
    }

    /**
    Calculates a curved autonomous path's angle by using the distance between the starting and ending point and the distance between the middle of the path and the height of the angular path
    <pre>
    .
                         
                          E
                      ~~~~~~~~~
                   ~~     +     ~~     
  /o----o\       ~~       + B     ~~       /o----o\
  |  (F) |  (2) ~~        +        ~~ (1)  |  (F) |
  \o----o/      ==========A==========      \o----o/ 
                \         |         /    
                 \        |        /
                  \       |       /
                   \      |D     / 
                    \     |     / 
                     \  __|__  / 
                      \/  C  \/
                       \  |  /
                        \ | /
                         \|/
            
    A = distanceA / =
    B = lengthB / +
    C = angle
    D = radius / |
    E = circumference / ~
    F = robot
    1 = starting position
    2 = ending position
    (diagram above isnt a hot air balloon fyi)
    </pre>

    * @param distanceA
    * @param lengthB
    * @return Angle of the path (how much the angular motors have to turn in order to acheive this path)
    * @see {@link #calculateCircleRadius(distanceA, lengthB)}
    */
    public double calculateAngleOfPath(double distanceA, double lengthB)
    {
        double radius = calculateCircleRadius(distanceA, lengthB);
        return 2 * (Math.toDegrees((Math.asin((distanceA/(2 * radius))))));
    }

 /**Restarts distance*/
    public void autoTurnSet()
    {
        currentDistance = 0;
    }

    /**
     * Using the {@link #swerveAutonomousEther} and motion magic, an autonomous angled path of motion can be achieved
     * @param totalDistance Length of curved path
     * @param thetaTurn Angle of curved path
     * @param RCWauto [-1, 1] For spinny, 0 for no spinny
     * @param mode Curve or Straight
     * @param turny Specific or Infinite
     * @param turnyAuto (if using specific for turny) angle that robot tries to keep when moving
     * @see {@link #swerveAutonomousEther(FWD, STR, RCW)}
     * @see {@link #updateMagicStraight()}
    */
    public void autoTurnUpdate(double totalDistance, double thetaTurn, double RCWauto, ETHERAUTO mode, ETHERRCW turny, double turnyAuto)
    {
        double RCWtemp = RCWauto;
        currentDistance = 
            (MkUtil.nativeToInches(topDriveLeft.getSelectedSensorPosition()) +
            MkUtil.nativeToInches(topDriveRight.getSelectedSensorPosition()) +
            MkUtil.nativeToInches(bottomDriveLeft.getSelectedSensorPosition()) + 
            MkUtil.nativeToInches(bottomDriveRight.getSelectedSensorPosition())) / 4;
        if(mode == ETHERAUTO.Curve)
        {
            FWDauto = Math.cos((-1 * turnDistance) + (2 * ((currentDistance/totalDistance)*turnDistance)) * Constants.kPi / 180);
            STRauto = Math.sin((-1 * turnDistance) + (2 * ((currentDistance/totalDistance)*turnDistance)) * Constants.kPi / 180);
            SmartDashboard.putNumber("STRauto", STRauto);
            SmartDashboard.putNumber("FWDauto", FWDauto);
        }
        else if(mode == ETHERAUTO.Straight)
        {
            FWDauto = Math.cos(thetaTurn);
            STRauto = Math.sin(thetaTurn);
            SmartDashboard.putNumber("STRauto", STRauto);
            SmartDashboard.putNumber("FWDauto", FWDauto);
        }
        if(turny == ETHERRCW.Specific)
        {
            RCWtemp = headerStraighter(turnyAuto);
        }
        swerveAutonomousEther(-FWDauto, STRauto, RCWtemp);
        //SmartDashboard.putNumber("AUTONASX", getNavx());
        SmartDashboard.putNumber("jihngnhjing", headerStraighter(turnyAuto));

        //swerveAutonomousEther(FWDauto, -STRauto, RCWtemp);
/*
        topTurnLeft.set(ControlMode.PercentOutput, topTurnLeftCalculateNative(MkUtil.degreesToNative(((currentDistance/totalDistance)*thetaTurn), TURN.greerRatio)));
        topTurnRight.set(ControlMode.PercentOutput, topTurnRightCalculateNative(MkUtil.degreesToNative(((currentDistance/totalDistance)*thetaTurn), TURN.greerRatio)));
        bottomTurnLeft.set(ControlMode.PercentOutput, bottomTurnLeftCalculateNative(MkUtil.degreesToNative(((currentDistance/totalDistance)*thetaTurn), TURN.greerRatio)));
        bottomTurnRight.set(ControlMode.PercentOutput, bottomTurnRightCalculateNative(MkUtil.degreesToNative(((currentDistance/totalDistance)*thetaTurn), TURN.greerRatio)));
 */
    }

    /**
     * Returns state of auto turn
     * @param  totalDistance Length of curved path (the same distance set in the {@link #setMagicStraight} function)
     * @return True if turning is done
     * @see {@link #setMagicStraight(setpoint)}
     */
    public boolean autoTurnIsDone(double totalDistance)
    {
        return Math.abs(totalDistance - currentDistance) < 0.5 && Math.abs(avgVelInches) < 0.1;
    }


    /**
     * Resets drive motors and sets motion magic velocity, acceleration, and distance
     * @param setpoint Distance (inches)
     * @param magicVelo Motion magic maximum velocity
     * @param magicAccel Motion magic maximum acceleration
     */
    public void setMagicStraight(double setpoint, double magicVelo, double magicAccel)
    {
        resetDrive();
        distance = setpoint;
        topDriveLeft.configMotionCruiseVelocity(magicVelo);
        topDriveRight.configMotionCruiseVelocity(magicVelo);
        bottomDriveLeft.configMotionCruiseVelocity(magicVelo);
        bottomDriveRight.configMotionCruiseVelocity(magicVelo);

        topDriveLeft.configMotionAcceleration(magicAccel);
        topDriveRight.configMotionAcceleration(magicAccel);
        bottomDriveLeft.configMotionAcceleration(magicAccel);
        bottomDriveRight.configMotionAcceleration(magicAccel);
    }

    /**
     * Resets turn motors and sets motion magic velocity, acceleration, and distance
     * @param setpoint Distance or angle idk anymore (inches, and the same distance set in the {@link #setMagicStraight} function). however, if you are turning without driving, use native units instead of inches
     * @see {@link #setMagicStraight(setpoint)}
     */
    public void setMagicTurn(double setpoint)
    {
        turnDistance = setpoint;
        topTurnLeft.configMotionCruiseVelocity(DRIVE.magicVelo);
        topTurnRight.configMotionCruiseVelocity(DRIVE.magicVelo);
        bottomTurnLeft.configMotionCruiseVelocity(DRIVE.magicVelo);
        bottomTurnRight.configMotionCruiseVelocity(DRIVE.magicVelo);

        topTurnLeft.configMotionAcceleration(DRIVE.magicAccel);
        topTurnRight.configMotionAcceleration(DRIVE.magicAccel);
        bottomTurnLeft.configMotionAcceleration(DRIVE.magicAccel);
        bottomTurnRight.configMotionAcceleration(DRIVE.magicAccel);
    }

    /**
     * Updates turn motors with magic
     * @param totalDistance The same distance set in the {@link #setMagicStraight} function
     * @see {@link #setMagicStraight(setpoint)}
     */
    public void updateMagicTurn(double totalDistance)
    {
        currentDistance = 
        (MkUtil.nativeToInches(topDriveLeft.getSelectedSensorPosition()) +
        MkUtil.nativeToInches(topDriveRight.getSelectedSensorPosition()) +
        MkUtil.nativeToInches(bottomDriveLeft.getSelectedSensorPosition()) + 
        MkUtil.nativeToInches(bottomDriveRight.getSelectedSensorPosition())) / 4;

        topTurnLeft.set(ControlMode.MotionMagic, MkUtil.degreesToNative((-1 * turnDistance) + (2 * ((currentDistance/totalDistance)*turnDistance)), TURN.greerRatio));
        topTurnRight.set(ControlMode.MotionMagic, MkUtil.degreesToNative((-1 * turnDistance) + (2 * ((currentDistance/totalDistance)*turnDistance)), TURN.greerRatio));
        bottomTurnLeft.set(ControlMode.MotionMagic, MkUtil.degreesToNative((-1 * turnDistance) + (2 * ((currentDistance/totalDistance)*turnDistance)), TURN.greerRatio));
        bottomTurnRight.set(ControlMode.MotionMagic, MkUtil.degreesToNative((-1 * turnDistance) + (2 * ((currentDistance/totalDistance)*turnDistance)), TURN.greerRatio));
        //SmartDashboard.putNumber("currentdist", currentDistance);
        //SmartDashboard.putNumber("WORK", (-1 * turnDistance) + (2 * ((currentDistance/totalDistance)*turnDistance)));
    }

    /**
     * Updates turn motors with magic if you arent turning while driving and have set the setpoint in {@link #setMagicTurn} to native units instead of a distance (inches)
     * @see {@link #setMagicTurn(setpoint)}
     */
    public void updateMagicTurnAlone()
    {
        topTurnLeft.set(ControlMode.MotionMagic, MkUtil.degreesToNative(turnDistance, TURN.greerRatio));
        topTurnRight.set(ControlMode.MotionMagic, MkUtil.degreesToNative(turnDistance, TURN.greerRatio));
        bottomTurnLeft.set(ControlMode.MotionMagic, MkUtil.degreesToNative(turnDistance, TURN.greerRatio));
        bottomTurnRight.set(ControlMode.MotionMagic, MkUtil.degreesToNative(turnDistance, TURN.greerRatio));
    }

 /**Updates drive motors with magic*/
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

        //SmartDashboard.putNumber("dist", distance);
        //SmartDashboard.putNumber("leftout", leftTopOutput);
    }

    /**
     * Returns state of motion magic
     * @return True if motion magic is done
     */
    public boolean isMagicStraightDone()
    {
        double err = distance - avgDistInches;              //TODO 0.1?? 
        return Math.abs(err) < 0.1 && Math.abs(avgVelInches) < 0.1; //0.5, 0.5
    }

    /**
     * same as {@link #isMagicStraightDone} but you can customize the distance variable
     * @param inchesEther distance of the ether
     * @return True if ether is done
     */
    public boolean isEtherMoveDone(double inchesEther)
    {
        double err = inchesEther - avgDistInches;              //TODO 0.1?? 
        return Math.abs(err) < 0.1 && Math.abs(avgVelInches) < 0.1; //0.5, 0.5
    }

    /**
     * Returns the state of percent turning (use if using {@link #updateMagicTurnAlone} and using native units instead of inches in {@link #setMagicTurn})
     * @return True if percent turning is done
     * @see {@link #updateMagicTurnAlone()}
     * @see {@link #setMagicTurn(setpoint)}
     */
    public boolean percentTurnDone()
    {
        double err = turnDistance - ((MkUtil.nativeToDegrees(topTurnLeft.getSelectedSensorPosition(), TURN.greerRatio) + MkUtil.nativeToDegrees(topTurnRight.getSelectedSensorPosition(), TURN.greerRatio) + MkUtil.nativeToDegrees(bottomTurnLeft.getSelectedSensorPosition(), TURN.greerRatio) + MkUtil.nativeToDegrees(bottomTurnRight.getSelectedSensorPosition(), TURN.greerRatio)) /4.0);
        return Math.abs(err) < 0.5;
    }

    /**Mode of the ether auto's path*/
    public enum ETHERAUTO
    {
        Straight, Curve
    }

    /**Mode of the ether auto's turn */
    public enum ETHERRCW
    {
        Specific, Forever
    }
    


    public void testDriveMotors(double speed, double time)
    {
        driveVelocity(speed, speed, speed, speed);
        if(time > 1)
        {
            driveErrorTest[0] = (driveErrorTest[0] + avgVelNative)/2;
            driveErrorTest[1] = (driveErrorTest[1] + avgVelNative)/2;
            
            driveErrorTest[2] = (driveErrorTest[2] + leftTopVelNative)/2;
            driveErrorTest[3] = (driveErrorTest[3] + rightTopVelNative)/2;
            driveErrorTest[4] = (driveErrorTest[4] + leftBottomVelNative)/2;
            driveErrorTest[5] = (driveErrorTest[5] + rightBottomVelNative)/2;

            driveErrorTest[6] = (driveErrorTest[6] + leftTopVelNative)/2;
            driveErrorTest[7] = (driveErrorTest[7] + rightTopVelNative)/2;
            driveErrorTest[8] = (driveErrorTest[8] + leftBottomVelNative)/2;
            driveErrorTest[9] = (driveErrorTest[9] + rightBottomVelNative)/2;
        }
        else
        {
            driveErrorTest[0] = 0;
            driveErrorTest[1] = speed;
            driveErrorTest[2] = 0;
            driveErrorTest[3] = 0;
            driveErrorTest[4] = 0;
            driveErrorTest[5] = 0;
            driveErrorTest[6] = speed;
            driveErrorTest[7] = speed;
            driveErrorTest[8] = speed;
            driveErrorTest[9] = speed;
        }
    }

    public void testAngleMotors(double angle, double time)
    {
        turnCalcPercent(angle, angle, angle, angle);
        if(time > 1)
        {
            turnErrorTest[0] = (turnErrorTest[0] + avgVelNative)/2;
            turnErrorTest[1] = (turnErrorTest[1] + avgVelNative)/2;
            
            turnErrorTest[2] = (turnErrorTest[2] + leftTopVelNative)/2;
            turnErrorTest[3] = (turnErrorTest[3] + rightTopVelNative)/2;
            turnErrorTest[4] = (turnErrorTest[4] + leftBottomVelNative)/2;
            turnErrorTest[5] = (turnErrorTest[5] + rightBottomVelNative)/2;

            turnErrorTest[6] = (turnErrorTest[6] + leftTopVelNative)/2;
            turnErrorTest[7] = (turnErrorTest[7] + rightTopVelNative)/2;
            turnErrorTest[8] = (turnErrorTest[8] + leftBottomVelNative)/2;
            turnErrorTest[9] = (turnErrorTest[9] + rightBottomVelNative)/2;
        }
        else
        {
            turnErrorTest[0] = 0;
            turnErrorTest[1] = angle;
            turnErrorTest[2] = 0;
            turnErrorTest[3] = 0;
            turnErrorTest[4] = 0;
            turnErrorTest[5] = 0;
            turnErrorTest[6] = angle;
            turnErrorTest[7] = angle;
            turnErrorTest[8] = angle;
            turnErrorTest[9] = angle;
        }
    }



    public TalonFX[] getDriveMotors()
    {
        return driveMotors;
    }

    public TalonFX[] getTurnMotors()
    {
        return turnMotors;
    }

    /**0 driveErrZero, 1 driveErrFull, 2-5 driveMotorErrZero, 6-9 driveMotorErrFull (topLeft, topRight, botLeft, botRight)*/
    private double[] driveErrorTest = new double[10];

    /**0 turnErrZero, 1 turnErrFull, 2-5 turnMotorErrZero, 6-9 turnMotorErrFull (topLeft, topRight, botLeft, botRight)*/
    private double[] turnErrorTest = new double[10];


    private TalonFX[] driveMotors = new TalonFX[6];

    private TalonFX[] turnMotors = new TalonFX[6];


    public void assignTalonArray()
    {
        driveMotors[0] = topDriveLeft;
        driveMotors[1] = topTurnRight;
        driveMotors[2] = bottomTurnLeft;
        driveMotors[3] = bottomTurnRight;

        turnMotors[0] = topTurnLeft;
        turnMotors[1] = topTurnRight;
        turnMotors[2] = bottomTurnLeft;
        turnMotors[3] = bottomTurnRight;
    }


    public double getError(ERROR type, int err)
    {
        switch(type)
        {
            //TODO no breaks, says its bad, idk tho lets hope it doesnt crash
            case DRIVE:
                return driveErrorTest[err];
            case TURN:
                return turnErrorTest[err];
            default:
                break;
        }
        return 69;
    }



    public enum ERROR
    {
        DRIVE, TURN;
    }

   /* private double driveErrorTestFromZero = 0;
    private double driveErrorTestFromFull = 0;

    private double driveTopLeftErrorTestFromZero = 0;
    private double driveTopRightErrorTestFromZero = 0;
    private double driveBottomLeftErrorTestFromZero = 0;
    private double driveBottomRightErrorTestFromZero = 0;

    private double driveTopLeftErrorTestFromFull = 0;
    private double driveTopRightErrorTestFromFull = 0;
    private double driveBottomLeftErrorTestFromFull = 0;
    private double driveBottomRightErrorTestFromFull = 0;


    private double turnErrorTestFromZero = 0;
    private double turnErrorTestFromFull = 0;

    private double turnTopLeftErrorTestFromZero = 0;
    private double turnTopRightErrorTestFromZero = 0;
    private double turnBottomLeftErrorTestFromZero = 0;
    private double turnBottomRightErrorTestFromZero = 0;

    private double turnTopLeftErrorTestFromFull = 0;
    private double turnTopRightErrorTestFromFull = 0;
    private double turnBottomLeftErrorTestFromFull = 0;
    private double turnBottomRightErrorTestFromFull = 0;
    */

    private static class InstanceHolder
    {
        private static final Drive mInstance = new Drive();
    } 
}
