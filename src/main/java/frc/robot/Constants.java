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
    public static double L = 29;

    //trackwidth (distance between the middle of the wheels on the width side)
    public static double W = 17.625; 

    public static double widthInch = 21;
    public static double heightInch = 32;

    public static double R = Math.sqrt(Math.pow(L, 2) + Math.pow(W, 2));

    public static class DRIVE
    {
        public static double greerRatio = 6.75;

        public static int topDriveRightCANID = 12;
        public static int bottomDriveRightCANID = 3;
        public static int topDriveLeftCANID = 9;
        public static int bottomDriveLeftCANID = 6;

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
        public static int topTurnLeftCANCoderCANID = 16;
        public static int topTurnRightCANCoderCANID = 14;
        public static int bottomTurnLeftCANCoderCANID = 15;
        public static int bottomTurnRightCANCoderCANID = 13;

        public static double deadband = 0.1;

        public static double greerRatio = 12.8;

        public static int topTurnLeftCANID = 4; 
        public static int topTurnRightCANID = 7;
        public static int bottomTurnLeftCANID = 2;
        public static int bottomTurnRightCANID = 1;

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
        public static double bottomLeftOffset = 117.94921875;
        public static double bottomRightOffset = -47.109375;

        //can be used to keep motors at a steady rate of power consumption
        public static double voltComp = 12;

        public static double kS = 0.66294; //0.4969;
        public static double maxVel = 21420;
        //TODO get max accel
        public static double maxAccel = 300; 
        public static double kV = 0.10986; //12 / maxVel;
    }

    public static class LIGHTS
    {
        public static int PWMPORT = 0; 
        public static int bufferNum = 100; 
    }

    public static class AUTO
    {                               //with encode option    without
        public static double turnKP = 0.0000063403;        //1.6261; //0.0000001;
        public static double turnKI = 0;
        public static double turnKD = 0.000000098857;//0.024918;  //0.0000000; 
        public static double turnMaxVelo = 0.001;  //1;
        public static double turnMaxAccel = 0.001;  //1;

        public static double driveKP = 0.0000001;
        public static double driveKI = 0;
        public static double driveKD = 0;
        public static double kMaxSpeedMetersPerSecond = 0.01; //.75 * DRIVE.maxNativeVelocity;
        public static double kMaxAccelerationMetersPerSecondSquared = 0.01; //2000;

        public static double heightMeters = MkUtil.inchesToMeters(heightInch);
        public static double widthMeters = MkUtil.inchesToMeters(widthInch);
        public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
        new Translation2d(heightMeters, widthMeters),
        new Translation2d(heightMeters, -widthMeters),
        new Translation2d(-heightMeters, widthMeters),
        new Translation2d(-heightMeters, -widthMeters));

        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}

