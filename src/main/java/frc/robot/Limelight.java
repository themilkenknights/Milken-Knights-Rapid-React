// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Limelight {
    
    private Limelight()
    {

    }

    public static Limelight getInstance()
    {
        return InstanceHolder.mInstance;
    }
/*
    a2 = y component of limelight   
    public double getInchesDistance(double a2)
    {
        //return = (h2-h1) / tan(a1+a2)
    }
*/
    private static class InstanceHolder
    {
        private static final Limelight mInstance = new Limelight();
    } 
}
