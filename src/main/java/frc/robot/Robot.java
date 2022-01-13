// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commands.DriveStr8;
import frc.robot.Constants.DRIVE;
import frc.robot.Constants.TURN;

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

   private Command m_autonomousCommand;
   private SendableChooser<AutoPosition> positionChooser = new SendableChooser<>();
   private ShuffleboardTab mTab = Shuffleboard.getTab("Match");
   private ComplexWidget positionChooserTab = mTab.add("Auto Chooser", positionChooser).withWidget(BuiltInWidgets.kSplitButtonChooser);
   
   public enum AutoPosition {
     LEFT, NOTHING
   }
   @Override
   public void robotInit() {
     
     //Shuffleboard.startRecording();
     Shuffleboard.selectTab("Match");
     positionChooser.addOption("Nothing", AutoPosition.NOTHING);
     positionChooser.setDefaultOption("Left Trench", AutoPosition.LEFT);
   }
 
   @Override
   public void robotPeriodic() {
     CommandScheduler.getInstance().run();
     Shuffle.getInstance().update();
   }
 
   @Override
   public void autonomousInit() {
     Shuffleboard.addEventMarker("Auto Init", EventImportance.kNormal);
     mDrive.resetDrive();
     switch (positionChooser.getSelected()) {
       case LEFT:
         m_autonomousCommand = new DriveStr8();
         break;
       case NOTHING:
         
         break;
     }
     if (m_autonomousCommand != null) {
       m_autonomousCommand.schedule();
     }
   }
 
   @Override
   public void autonomousPeriodic() {
     mDrive.driveUpdate();
   }
 
   @Override
   public void teleopInit() {
     Shuffleboard.addEventMarker("Teleop Init", EventImportance.kNormal);
     if (m_autonomousCommand != null) {
       m_autonomousCommand.cancel();
     }
     mDrive.resetDrive();
   }
 
 
  @Override
  public void teleopPeriodic() {
    mDrive.driveUpdate();
    

     
    one = (xbox.getRawAxis(1) - DRIVE.deadband) / (1 - DRIVE.deadband);
    two = (xbox.getRawAxis(0) - DRIVE.deadband) / (1 - DRIVE.deadband);
    three = (xbox.getRawAxis(5) - TURN.deadband) / (1 - TURN.deadband);
      
      if(Math.abs(xbox.getRawAxis(1)) < 0.1)
      {
        one = 0;
      }
      if(Math.abs(xbox.getRawAxis(0)) < 0.4)
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
        mDrive.turnPercent(0,0,0,0);
        mDrive.drivePercent(0);
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
