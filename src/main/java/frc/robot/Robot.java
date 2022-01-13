// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

   private Drive mDrive = Drive.getInstance();
   private XboxController xbox = new XboxController(0);
   private double one;
   private double two;
   private double three;

  @Override
  public void robotInit() {}

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    mDrive.driveUpdate();
    

      one = (xbox.getRawAxis(1));
      two = (xbox.getRawAxis(0));
      three = (xbox.getRawAxis(5));
      
      if(Math.abs(xbox.getRawAxis(1)) < 0.1)
      {
        one = 0;
      }
      if(Math.abs(xbox.getRawAxis(0)) < 0.1)
      {
        two = 0;
      }
      if(Math.abs(xbox.getRawAxis(5)) < 0.1)
      {
        three = 0;
      }

      
      
      if(one != 0 || two != 0 || three != 0)
      {
        mDrive.etherSwerve(one/3,two/3,three/3);
      }
      else
      {
        mDrive.turnPercent(0);
      }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}
