// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//!                   sigma grindset rule #29534 - always keep your accidental imports, even if you never use them

import java.awt.MouseInfo;
import java.util.List;

import javax.swing.event.MouseInputListener;
import javax.swing.plaf.basic.BasicDesktopIconUI.MouseInputHandler;
import javax.swing.plaf.basic.BasicTabbedPaneUI.MouseHandler;
import javax.xml.crypto.dsig.keyinfo.KeyInfo;

import org.w3c.dom.events.MouseEvent;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Commands.DriveStr8;
import frc.robot.Constants.AUTO;
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
   private Shooter mShoot = Shooter.getInstance();
   private XboxController xbox = new XboxController(0);


    /**
     * forward movement axis
     */ 
   private double fwd;
    /**
     * strafing movement axis
     */ 
   private double str;
    /**
     * rotational movement axis
     */ 
   private double rcw;

   private Command m_autonomousCommand;
   private SendableChooser<AutoPosition> positionChooser = new SendableChooser<>();
   private ShuffleboardTab mTab = Shuffleboard.getTab("Match");
   private ComplexWidget positionChooserTab = mTab.add("Auto Chooser", positionChooser).withWidget(BuiltInWidgets.kSplitButtonChooser);
   
   private SendableChooser<veloch> veloshufflething = new SendableChooser<veloch>();
   
   /**
    * for wpi swerve and auto
    */
   private RobotContainer m_robotContainer;
   
    /**
     * states of autonomous
     */ 
   public enum AutoPosition {
     LEFT, NOTHING
   }

   /**
    * states of shooter speed
    */
   public enum veloch
   {
     veloOne, veloTwo, veloThree
   }

   /**
    * for slider widget
    */
   private double slider;

   /**
    * for motor volt testing
    */
   private double volts;

   /**
    * baby toggle fast
    */
   private boolean toggleFastOn = false;

   /**
    * baby pressed fast
    */
   private boolean toggleFastPressed = false;


   /**
    * baby toggle slow
    */
   private boolean toggleSlowOn = false;

   /**
    * baby pressed slow
    */
   private boolean toggleSlowPressed = false;

   /**
    * constant that divides speed (baby control)
    */
   private double spee = 0;

   /**
    * setpoint variable for shooting
    */
   private int velo = 0;

   @Override
   public void robotInit() {
     
     //Shuffleboard.startRecording();
     m_robotContainer = new RobotContainer();
     mTab.add("velochoose", veloshufflething).withWidget(BuiltInWidgets.kSplitButtonChooser);
     Shuffleboard.selectTab("Match");
     positionChooser.addOption("Nothing", AutoPosition.NOTHING);
     positionChooser.setDefaultOption("Left Trench", AutoPosition.LEFT);
     veloshufflething.addOption("spee1", veloch.veloOne);
     veloshufflething.addOption("spee2", veloch.veloTwo);
     veloshufflething.addOption("spee3", veloch.veloThree);
     veloshufflething.setDefaultOption("spee1", veloch.veloOne);
      // etc.
    



      


   }
 
   @Override
   public void robotPeriodic() {
     CommandScheduler.getInstance().run();
     Shuffle.getInstance().update();
   }
 
   @Override
   public void autonomousInit() {
     Shuffleboard.addEventMarker("Auto Init", EventImportance.kNormal);
     //m_robotContainer.resetPID();
     mDrive.resetDrive();
     mDrive.resetNavx();
     switch (positionChooser.getSelected()) {
       case LEFT:
         m_autonomousCommand = m_robotContainer.getAutonomousCommand();
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
     //mDrive.driveUpdate();
   }
 
   @Override
   public void teleopInit() {
     mDrive.encoderZero();
     //TODO need to see if drive initiated with code runned, problem if not runned
     //TODO it runs everything except encoder reset i guess, but also need to set offsets back negative see if work
     Shuffleboard.addEventMarker("Teleop Init", EventImportance.kNormal);
     if (m_autonomousCommand != null) {
       m_autonomousCommand.cancel();
     }
     mDrive.resetDrive();
     SmartDashboard.putNumber("slider", 0);
     switch (veloshufflething.getSelected()) {
      case veloOne:
        velo = 0;
        break;
      case veloTwo:
        velo = 10000;
        break;
       
      case veloThree:
        velo = 14000;
        break;
    }
   }
 
 
  @Override
  public void teleopPeriodic() {
    
    mDrive.driveUpdate();
    mShoot.shooterUpdate();
    updateFastToggle();
    updateSlowToggle();

    //MouseInfo.getPointerInfo();
    //TODO do something with mouse?

    fwd = (xbox.getRawAxis(1) - DRIVE.deadband) / (1 - DRIVE.deadband);
    str = (xbox.getRawAxis(0) - DRIVE.deadband) / (1 - DRIVE.deadband);
    rcw = (xbox.getRawAxis(4) - TURN.deadband) / (1 - TURN.deadband);
      
      if(Math.abs(xbox.getRawAxis(1)) < 0.1)
      {
        fwd = 0;
      }
      if(Math.abs(xbox.getRawAxis(0)) < 0.1)
      {
        str = 0;
      }
      if(Math.abs(xbox.getRawAxis(4)) < 0.1)
      {
        rcw = 0;
      }


      if(fwd != 0 || str != 0 || rcw != 0)
      {
        mDrive.etherSwerve(-fwd/spee,str/spee,rcw/spee);
      }
      else if(xbox.getAButton())
      {
        mDrive.turnCalcPercent(0, 0, 0, 0);
      }
      else if(xbox.getBButton())
      {
        mDrive.resetDrive();
      }
      else if(xbox.getXButton())
      {
        mDrive.resetNavx();
      }
      else if(xbox.getYButton())
      {
        mDrive.encoderZero();
      }
      else if(xbox.getRawAxis(2) > 0)
      {
        //mShoot.setShooterNativeVeloctiy(velo);
        mShoot.setShooterPercent(xbox.getRawAxis(2));
      }
      else
      {
        mDrive.turnPercent(0,0,0,0);
        mShoot.setShooterPercent(0);
        mDrive.drivePercent(0,0,0,0);
      }


      if(toggleFastOn){
        // Do something when toggled on
        spee = 1;
      }
      else if(toggleSlowOn){
        spee = 7;
      }
      else{
          // Do something when toggled off
        spee = 3;
      }

      //SmartDashboard.putNumber("x", MkUtil.metersToInches(mOdo.getX()));
      //SmartDashboard.putNumber("y",  MkUtil.metersToInches(mOdo.getY()));
 
      slider = SmartDashboard.getNumber("slider", 0);
      SmartDashboard.putNumber("spee", spee);
      SmartDashboard.putNumber("velo", velo);
    }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  /**
   * updates state of fast toggle for driving
   */
  public void updateFastToggle()
  {
      if(xbox.getPOV() == 0){
          if(!toggleFastPressed){
              toggleFastOn = !toggleFastOn;
              toggleSlowOn = false;
              //TODO ^^ might not work
              toggleFastPressed = true;
          }
      }
      else{
          toggleFastPressed = false;
      }
  }

  /**
   * updates state of slow toggle for driving
   */
  public void updateSlowToggle()
  {
      if(xbox.getPOV() == 180){
          if(!toggleSlowPressed){
              toggleSlowOn = !toggleSlowOn;
              toggleFastOn = false;
              //TODO ^^ might not work
              toggleSlowPressed = true;
          }
      }
      else{
          toggleSlowPressed = false;
      }
  }
}
