// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/** Add your docs here. */
public final class Constants {
    public static double kPi = 3.14159265359;
    public static double oneEncoderRotation = 2048;

    // see link for more info https://www.chiefdelphi.com/t/paper-4-wheel-independent-drive-independent-steering-swerve/107383 
    //                        (Derivation of Inverse Kinematics for Swerve, page 4)

    //wheelbase (distance between the middle of the wheels on the length side)
    public static double L = 22.57; //29;

    //trackwidth (distance between the middle of the wheels on the width side)
    public static double W = 22.57; //17.625; 

    public static double widthInch = 29; //21;
    public static double heightInch = 29; //32;

    public static double R = Math.sqrt(Math.pow(L, 2) + Math.pow(W, 2));

    public static class DRIVE
    {
        public static int magicSCurve = 6;
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

        public static double deadband = 0.1;

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
        public static double turnKP = 0.00008;//0.00008
        public static double turnKI = 0;
        public static double turnKD = 0.00000001; 
        public static double turnKF = 0;

        public static double magicturnKP = 0.085;//0.00008
        public static double magicturnKI = 0;
        public static double magicturnKD = 0.00000001; 
        public static double magicturnKF = 0;

        // (CANCoder) in degrees, 180 to -180 
        public static double topLeftOffset = 70.9277343;
        public static double topRightOffset = 107.138671875;
        //TODO negative offset?
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

        public static int shootleftCANID = 19; //;
        public static int shootrightCANID = 20; //;
        public static int beltCANID = 11; //

        public static int shootKP = 0;
        public static int shootKI = 0;
        public static int shootKD = 0;
        public static int shootKF = 0;

        public static double beltGreerRatio = 0;

        public static double maxVelo = 0;

        public static double voltComp = 12;
;
     
    }

<<<<<<< HEAD
    public static class INTAKE 
    {
        public static double voltComp = 12;

        public static int INTAKECANID = 14; 
        public static int INTAKEROLLERCANID = 13; 

        public static double intakeGreerRatio = 0;
        public static double rollersGreerRatio = 0;

        public static double intakeKP = .002;
        public static double intakeKI = 0;
        public static double intakeKD = 0.001;
        public static double intakeKF = 1; //TODO may not need pid if bang bang limit swtich


    }

=======
>>>>>>> parent of 417e8ac (added intake and shooter class)
    public static class LIMELIGHT
    {
        public static double angleAboveHorizontal = 0;
        public static double height = 0;
        public static double lowerGoalHeight = 0;
        public static double upperGoalHeight = 0;
    }

    public static class LIGHTS
    {
        public static int PWMPORT = 0; 
        public static int bufferNum = 100; 
    }

    public static class AUTO
    {                                                                                   //with encode option    without
        public static double moduleTurnKP = 0.063403; //0.63403; //0.063403;            //0.0000063403;        //1.6261;      //0.0000001;
        public static double moduleTurnKI = 0;
        public static double moduleTurnKD = 0.00098857; //0.0098857;// 0.00098857;     //0.000000098857;     //0.024918;       //0.0000000; 

        public static double moduleDriveKP = 0.03; //0.0000001;
        public static double moduleDriveKI = 0;
        public static double moduleDriveKD = 0.001;

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
        //TODO also see if dividing by two helps and setting it to the actual wheelbase and trackwdith

        
        
        public static final double maxModuleTurnVelo = kPi * 2;
        public static final double maxModuleTurnAccel = kPi * 2;
        
        public static final double maxModuleDriveVelo = 2; //2;
        public static final double maxModuleDriveAccel = 2; //2;
        


        public static final double maxAutoTurnVelo = kPi * 2;
        public static final double maxAutoTurnAccel = kPi * 2;
        
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

