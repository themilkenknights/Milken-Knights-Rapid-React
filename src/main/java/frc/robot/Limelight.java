// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** class containing calculations of the lime fruit when shooting */
public class Limelight {
    /*
         _____________________________
        [         ___________         ]
        |    .   /           \   .    |
        |   ::  /             \  ::   |
        |  ::: |     ( 0 )     | :::  |
        |   ::  \             /  ::   |
        |    .   \___________/   .    |
        [_____________________________]
                 eye of sauron
    */
    private Limelight()
    {

    }

    public static Limelight getInstance()
    {
        return InstanceHolder.mInstance;
    }
/*
https://docs.limelightvision.io/en/latest/cs_estimating_distance.html#using-a-fixed-angle-camera
    public double getInchesDistance(double ty)
    {
        //return = (h2-h1) / tan(a1+a2)
        return (LIMELIGHT.upperGoalHeight - LIMELIGHT.heightLime) / (Math.tan(LIMELIGHT.angleAboveHorizontal + ty));
    }
*/
    private static class InstanceHolder
    {
        private static final Limelight mInstance = new Limelight();
    } 
}
