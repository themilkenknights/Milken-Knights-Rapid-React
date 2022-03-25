// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.miscellaneous;

import frc.robot.Drive;
import frc.robot.Constants.DRIVE;

/** Add your docs here. */
public class Simulation {
    private Odometry mOdo = Odometry.getInstance();
    private Drive mDrive = Drive.getInstance();
    private double distanceA = 30;
    private double lengthB = 30;
    private double distance = mDrive.calculateArcOfPath(distanceA, lengthB);
    private double angle = mDrive.calculateAngleOfPath(distanceA, lengthB);
    private double loopTime = 0.02;
    //https://www.omnicalculator.com/physics/acceleration

    public static Odometry getInstance()
    {
        return InstanceHolder.mInstance;
    }

    public void updateVelocity()
    {
        //or do tan of distances x and y to find actual distance of motor
        double tesDist = Math.sqrt(Math.pow(mOdo.getX(), 2) + Math.pow(mOdo.getY(), 2));
        double testRot = (tesDist / distance) * angle;
        double rot = (((mOdo.getX() / distanceA) + (mOdo.getY() / lengthB)) / 2) * angle;
        mOdo.drive(DRIVE.maxNativeVelocity * Math.cos(angle), DRIVE.maxNativeVelocity * Math.cos(angle), rot, true);
    }

    private static class InstanceHolder
    {
        private static final Odometry mInstance = new Odometry();
    } 
}
