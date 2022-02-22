// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/** class containing calculations of the lime fruit when shooting */
public class Limelight {

  private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private final NetworkTableEntry tx = table.getEntry("tx");
  private final NetworkTableEntry ty = table.getEntry("ty");
  private final NetworkTableEntry led = table.getEntry("ledMode");
  private final NetworkTableEntry tv = table.getEntry("tv");
  private final NetworkTableEntry pipeline = table.getEntry("UR_MUM");
/** <pre>".
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
</pre>
*/
    private Limelight()
    {
        table.getEntry("UR_MOM").setValue(0);
        //table.getEntry("ledMode").setValue(3);
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
}
