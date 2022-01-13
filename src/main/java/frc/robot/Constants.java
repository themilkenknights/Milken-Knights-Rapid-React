// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public final class Constants {
    public static double kPi = 3.14159265359;
    public static double oneEncoderRotation = 2048;

    public static double L = 29;
    public static double W = 17.625;
    public static double R = Math.sqrt(Math.pow(L, 2) + Math.pow(W, 2));

    public static class DRIVE
    {
        public static double greerRatio = 6.75;

        public static int topDriveRightCANID = 12;
        public static int bottomDriveRightCANID = 3;
        public static int topDriveLeftCANID = 9;
        public static int bottomDriveLeftCANID = 6;

        public static double maxNativeVelocity = 21000; 

        public static double driveKP = 0.21;
        public static double driveKI = 0;
        public static double driveKD = 0 * driveKP; 
        public static double driveKF = 1023.0 / maxNativeVelocity;

        public static double maxInchesVelocity = MkUtil.nativeToInches(maxNativeVelocity);

        public static double magicVel = .75 * maxNativeVelocity;
        public static double magicAccel = 2000;

        public static double kWheelDiameterInches = 4; 
        public static double kWheelCircumference = kWheelDiameterInches * kPi;

        public static double voltComp = 12;

        public static double deadband = 0.1;
    }

    public static class TURN
    {
        public static int topTurnLeftCANCoderCANID = 16;
        public static int topTurnRightCANCoderCANID = 14;
        public static int bottomTurnLeftCANCoderCANID = 15;
        public static int bottomTurnRightCANCoderCANID = 13;

        public static double deadband = 0.1;
        public static double maxVel = 2000; 
        public static double maxAccel = 300; 

        public static double greerRatio = 12.8;

        public static int topTurnLeftCANID = 4; 
        public static int topTurnRightCANID = 7;
        public static int bottomTurnLeftCANID = 2;
        public static int bottomTurnRightCANID = 1;

        public static double turnKP = 0.00008;
        public static double turnKI = 0;
        public static double turnKD = 0.00000001; 
        public static double turnKF = 0;

        public static double topLeftOffset = 0;
        public static double topRightOffset = 0;
        public static double bottomLeftOffset = 0;
        public static double bottomRightOffset = 0;

        public static double voltComp = 12;

    }

    public static class LIGHTS
    {
        public static int PWMPORT = 0; 
        public static int bufferNum = 100; 
    }

    public static class CANNON
    {
        //TODO fix all of these
        public static int solenoidOneID = 0;
        public static int solenoidTwoID = 0;
        public static int compressorID = 0;
        public static int genevaConventionCANID = 0;
        public static int trunnionCANID = 0;

        public static double voltComp = 12;
        
        public static double genevaKP = 0.000001;
        public static double genevaKI = 0;
        public static double genevaKD = 0;
        public static double genevaKF = 0;

        public static double trunnionKP = 0.000001;
        public static double trunnionKI = 0;
        public static double trunnionKD = 0;
        public static double trunnionKF = 0;

        public static double trunnionGreerRatio = 0;

        public static double maxRotTalonSRX = 0;

        //!tester constants for feedforward

        public static double horizontalPosition = 0;
        public static double weight = 0;
        public static double cannonDistance = 0;
    }
}

