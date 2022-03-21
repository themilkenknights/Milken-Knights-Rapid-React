// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**Class containing calculations of the lime fruit when shooting */
public class Limelight {

  private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

/**Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)*/
  private final NetworkTableEntry tx = table.getEntry("tx");

/**Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees)*/
  private final NetworkTableEntry ty = table.getEntry("ty");

/**Sets limelight’s LED state*/
  private final NetworkTableEntry led = table.getEntry("ledMode");

/**Whether the limelight has any valid targets (0 or 1)*/
  private final NetworkTableEntry tv = table.getEntry("tv");

  private final NetworkTableEntry pipeline = table.getEntry("UR_MUM");

    private Limelight()
    {
        table.getEntry("UR_MOM").setValue(0);
        table.getEntry("ledMode").setValue(3);
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

 /**Toggles limelight on and off*/
    public void limelightToggle()
    {
        if (led.getDouble(0.0) == 1) {
            table.getEntry("ledMode").setValue(3);
          } else {
            table.getEntry("ledMode").setValue(1);
          }    

        
    }

    private static class InstanceHolder
    {
        private static final Limelight mInstance = new Limelight();
    } 





















/*
        ███████████████████████████████        
        █         ___________         █        
        █    .   /           \   .    █        
        █   ::  /             \  ::   █        
        █  ::: |     ( 0 )     | :::  █        
        █   ::  \             /  ::   █        
        █    .   \___________/   .    █        
        █                             █        
        ███████████████████████████████   
 
                 Eye Of Sauron
          He's looking for his bearing  
*/
}
