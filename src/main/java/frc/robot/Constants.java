// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/** 
 * @param voltComp voltage compensation value, usually 12 but user can define their own voltage compensation for each mechanism
 * @param velocityMeasAmount (I use 16, but user can define their own velocity window amount in each mechanism) windowSize Number of samples in the rolling average of velocity measurement. Valid values are 1,2,4,8,16,32. If another value is specified, it will truncate to nearest support value.
 * @param greerRatio gear ratio, pun intended
 * <h2> Status Measurments <h2/>
 * @param statusOneMeas
 * @param statusTwoMeas Feedback for selected sensor on primary PID[0].
 * <h2> SYSID Values <h2/>
 * @param kS  is the voltage needed to overcome the motor’s static friction, or in other words to just barely get it moving; it turns out that this static friction (because it’s, well, static) has the same effect regardless of velocity or acceleration. That is, no matter what speed you’re going or how fast you’re accelerating, some constant portion of the voltage you’ve applied to your motor (depending on the specific mechanism assembly) will be going towards overcoming the static friction in your gears, bearings, etc; this value is your kS. Note the presence of the signum function, because friction force always opposes the direction-of-motion.
 * @param kV describes how much voltage is needed to hold (or “cruise”) at a given constant velocity while overcoming the electromagnetic resistance in the motor and any additional friction that increases with speed (known as viscous drag). The relationship between speed and voltage (at constant acceleration) is almost entirely linear (with FRC® components, anyway) because of how permanent-magnet DC motors work.
 * @param kA describes the voltage needed to induce a given acceleration in the motor shaft. As with kV, the relationship between voltage and acceleration (at constant velocity) is almost perfectly linear for FRC components.
 */
public final class Constants {

    public static double kPi = 3.14159265359;
    
    /*** falcon encoder rotation*/
    public static double oneEncoderRotation = 2048;

    // see link for more info https://www.chiefdelphi.com/t/paper-4-wheel-independent-drive-independent-steering-swerve/107383 
    //                        (Derivation of Inverse Kinematics for Swerve, page 4)

    /*** wheelbase (distance between the middle of the wheels on the length side)*/
    public static double L = 22.57; //29;

    /*** trackwidth (distance between the middle of the wheels on the width side)*/
    public static double W = 22.57; //17.625; 

    public static double widthInch = 29; //21;
    public static double heightInch = 29; //32;

    public static double R = Math.sqrt(Math.pow(L, 2) + Math.pow(W, 2));

    public static class DRIVE
    {
        public static int magicSCurve = 6;

        //need to research these, might be eating cpu like chrome eats ram
        public static int velocityMeasAmount = 16;
        public static int statusOneMeas = 25;
        public static int statusTwoMeas = 25;

        public static double greerRatio = 6.75;

        public static int topDriveRightCANID = 5; //12;
        public static int bottomDriveRightCANID = 7; //3;
        public static int topDriveLeftCANID = 3; //9;
        public static int bottomDriveLeftCANID = 2; //6;

        public static double maxNativeVelocity = 21640; 

        //use team 3244's slideshow on motion magic to get pidf values
        public static double driveKP = 0.21;
        public static double driveKI = 0;
        public static double driveKD = 0 * driveKP; 
        public static double driveKF = 1023.0 / maxNativeVelocity;

        public static double maxInchesVelocity = MkUtil.nativeToInches(maxNativeVelocity);

        public static double magicVel = .75 * maxNativeVelocity;
        public static double magicAccel = 2000;

        public static double kWheelDiameterInches = 4; 
        public static double kWheelCircumference = kWheelDiameterInches * kPi;

        //can be used to keep motors at a steady rate of power consumption
        public static double voltComp = 12;

        //xbox controllers be hella sensative
        public static double deadband = 0.1;

        //see sysid and stuff documentation
        public static double kS = 0.5111;
        public static double kV = 12/maxNativeVelocity;
    }

    public static class TURN
    {
        public static int velocityMeasAmount = 16;
        public static int statusOneMeas = 25;
        public static int statusTwoMeas = 25;

        public static int topTurnLeftCANCoderCANID = 16; //16;
        public static int topTurnRightCANCoderCANID = 18; //14;
        public static int bottomTurnLeftCANCoderCANID = 15; //15;
        public static int bottomTurnRightCANCoderCANID = 17; //13;

        public static double deadband = 0.1;

        public static double greerRatio = 12.8;

        public static int topTurnLeftCANID = 4; //4; 
        public static int topTurnRightCANID = 6; //7;
        public static int bottomTurnLeftCANID = 1; //2;
        public static int bottomTurnRightCANID = 8; //1;

        //got these values by guessing and praying
        //TODO set all to 0, find kF, then find kP
        //TODO or dont use motor control loop, rather use calculation methods
        public static double turnKP = 0.00008;//0.00008;//0.00008
        public static double turnKI = 0;
        public static double turnKD = 0.00000001; // turnKP * 0.001; 
        public static double turnKF = 0;

        public static double turnTopLeftKF = 0;
        public static double turnTopRightKF = 0;
        public static double turnBottomLeftKF = 0;
        public static double turnBottomRightKF = 0;

        public static double magicturnKP = 0.085;//0.00008
        public static double magicturnKI = 0;
        public static double magicturnKD = 0.00000001; 
        public static double magicturnKF = 0;

        // (CANCoder) in degrees, 180 to -180 
        public static double topLeftOffset = 70.9277343;
        public static double topRightOffset = 107.138671875;
        //// negative offset?
        //i dont think so? idk
        public static double bottomLeftOffset = -117.94921875; //+
        public static double bottomRightOffset = 47.109375;  //-

        //can be used to keep motors at a steady rate of power consumption
        public static double voltComp = 12;

        public static double kS = 0.66294; //0.4969;
        public static double maxVel = 21420;
        //TODO get max accel
        public static double maxAccel = 300; 
        public static double kV = 0.10986; //12 / maxVel;
    }

    public static class SHOOT
    {
        public static boolean leftFlipped = false;

        public static int velocityMeasAmount = 16;
        public static int statusOneMeas = 25;
        public static int statusTwoMeas = 25;

        public static int shootLeftCANID = 19;
        public static int shootRightCANID = 20;
        
        public static double shootKP = 0.55; //0.64; //0.29559; //0.19559;
        public static double shootKI = 0;
        public static double shootKD = 0;
        public static double shootKF = 0;

        public static double kS = 0;
        public static double kV = 0;
        public static double kA = 0;
        //TODO may not need or use since sysid is kinda shit ngl

        //TODO need to account for battery and shit, battery affects max rpm
        public static double maxVelo = 18900; //17800; //16600;
        public static double minVelo = 0; //TODO may not need this if using sysid and integrated pid

        public static double voltComp = 12;

        public static double maxError = 1580;
    }

    public static class HOOD
    {
        public static double voltComp = 12;
        
        public static int hoodCANID = 0;

        public static double maxOutput = 0.25;
        public static double hoodKP = 0.15;
    }

    public static class ELEVATOR 
    {
        public static double voltComp = 12;
        
        public static int elevatorCANID = 11;

        public static double elevatorGreerRatio = 0;
    }

    public static class INTAKE 
    {
        public static double voltComp = 12;

        public static boolean leftFlipped = true; //TODO see if this works

        public static int intakeLeftCANID = 9; //14;
        public static int intakeRightCANID = 696969;
        public static int rollersCANID = 13; //13;

        public static double intakeGreerRatio = 0;
        public static double rollersGreerRatio = 0;

        public static double intakeKP = 0;
        public static double intakeKI = 0;
        public static double intakeKD = 0;
        public static double intakeKF = 0; //// may not need pid if bang bang limit swtich
                                           //not enough time to tune pidf for intake, bang bang is faster

        public static double timeIntake = 0; //// may not need if bang bang limit switch 
                                             //limit switches are a no go, not enough time

        /***how much it needs to rotate to move into out position*/
        public static double intakeRotationsNative = 0;
        /***threshold for intake rotations, since it wont be accurate*/
        public static double intakeOutThreshold = 0;
        /***if its less than this then stop rotating bang bang*/
        public static double intakeInMaxError = 0;

        public static double intakeBangBangSpeed = 0.5; //TODO may need to tune this if its too fast/slow
    }

    public static class CLIMBER
    {
        public static int velocityMeasAmount = 16;
        public static int statusOneMeas = 25;
        public static int statusTwoMeas = 25;

        public static int telescopeArmLeftCANID = 0;
        public static int telescopeArmRightCANID = 0;
        public static int rotateArmLeftCANID = 0;
        public static int rotateArmRightCANID = 0;

        public static double telescopeGreerRatio = 0;
        public static double rotateGreerRatio = 0;

        public static double telescopeKP = 0;
        public static double telescopeKI = 0;
        public static double telescopeKD = 0;
        public static double telescopeKF = 0;

        public static double rotateKP = 0;
        public static double rotateKI = 0;
        public static double rotateKD = 0;
        public static double rotateKF = 0;

        public static double voltComp = 12;

        public static boolean telescopeLeftFlipped = false;
        public static boolean rotateLeftFlipped = false;

        public static int telescopeMagicSCurve = 0;
        public static int rotateMagicSCurve = 0;

        //how many rotations native until high point
        public static double telescopeHighPointNative = 0;
        public static double rotateHighPointNative = 0;

        public static double telescopeNativePerInch = 0;
        //TODO try to find constant of slope of native per inch?

        public static double telescopeTimeHighPoint = 0;
        public static double rotateTimeHighPoint = 0;
        //TODO find after finding total rotations to high and testing, then time for extra precaution
    }

    public static class LIMELIGHT
    {
        //              inches
        
        public static double angleAboveHorizontal = 0;
        public static double heightLime = 0;
        //public static double lowerGoalHeight = 0; lime cant see when low goal
        public static double upperGoalHeight = 0;

        public static double closestDistanceHoodAngleLow = 0;
        public static double closestDistanceRPMLow = 0;
        
        public static double closestDistanceHoodAngleHigh = 0;
        public static double closestDistanceRPMHigh = 0;
    }

    public static class LIGHTS
    {
        public static int PWMPORT = 0; 
        public static int bufferNum = 100; 
    }

    public static class BUTTONS 
    {
        public static int forwardAxis = 1;
        public static int strafeAxis = 0;
        public static int spinAxis = 4;
        public static int intakeup = 2;
        public static int intakedown = 7;
        public static int hoodFowardButton = 90; //POV 90
        public static int hoodBackwardButton = 270; //POV 180
        public static int rollersForwardButton = 5;
        public static int rollersBackwardButton = 3;
        public static int elevatorForwardButton = 4;
        public static int elevatorBackwardButton = 6;
        public static int shooterButton = 1;
        public static int limelightButton = 8;

        //add climber when climber gets added

        public static int babySpeedFastAngle = 0;
        public static int babySpeedToddlerAngle = 180;
    }

    public static class AUTO
    {                                                                                   //with encode option    without
        //actual module pid
        public static double moduleTurnKP = 0.063403; //0.63403; //0.063403;            //0.0000063403;        //1.6261;      //0.0000001;
        public static double moduleTurnKI = 0;
        public static double moduleTurnKD = 0.00098857; //0.0098857;// 0.00098857;     //0.000000098857;     //0.024918;       //0.0000000; 
        //TODO fix pids if using wpi auto
        // why is making pid values a god damn guessing game

        //actual drive module pid
        public static double moduleDriveKP = 0.03; //0.0000001;
        public static double moduleDriveKI = 0;
        public static double moduleDriveKD = 0.001;

        //auto controlling pid
        public static double turnSwerveControlKp = 0.2;
        public static double driveSwerveControlKpY = 0.2;
        public static double driveSwerveControlKpX = 0.2;

        public static double heightMeters = MkUtil.inchesToMeters(L / 2);
        public static double widthMeters = MkUtil.inchesToMeters(W / 2);

        public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
        new Translation2d(heightMeters, widthMeters),
        new Translation2d(heightMeters, -widthMeters),
        new Translation2d(-heightMeters, widthMeters),
        new Translation2d(-heightMeters, -widthMeters));
        //// also see if dividing by two helps and setting it to the actual wheelbase and trackwdith
        //i think it works?
        
        //actual drive module stats
        public static final double maxModuleTurnVelo = kPi * 2;
        public static final double maxModuleTurnAccel = kPi * 2;
        
        //actual drive module stats
        public static final double maxModuleDriveVelo = 2; //2;
        public static final double maxModuleDriveAccel = 2; //2;
        

        //for turning constraints
        public static final double maxAutoTurnVelo = kPi * 2;
        public static final double maxAutoTurnAccel = kPi * 2;
        
        //for trajectory config
        public static final double maxAutoDriveVelo = 5; //2;
        public static final double maxAutoDriveAccel = 5; //2;


        public static final double maxDriveVelo = 5.0;

        //TODO test these for velocity 
        public static double autoVeloDriveKP = 0.21;
        public static double autoVeloDriveKI = 0;
        public static double autoVeloDriveKD = 0 * autoVeloDriveKI; 
        public static double autoVeloDriveKF = 1023.0 / DRIVE.maxNativeVelocity;


        
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            maxAutoTurnVelo, maxAutoTurnAccel);

        public static double nativeToMetersPerSec = (10 * (MkUtil.inchesToMeters(DRIVE.kWheelDiameterInches) * kPi))/(DRIVE.greerRatio * 2048);
    }
}

