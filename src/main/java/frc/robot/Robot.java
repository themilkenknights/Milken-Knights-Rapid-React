// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.lang.model.util.ElementScanner6;

//!                   sigma grindset rule #29534 - always keep your accidental imports, even if you never use them




import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commands.DriveStr8;
import frc.robot.Constants.BUTTONS;
import frc.robot.Constants.DRIVE;
import frc.robot.Constants.TURN;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  
   private Drive mDrive = Drive.getInstance();
   private Shooter mShoot = Shooter.getInstance();
   private Elevator mElevator = Elevator.getInstance();
   private Intake mIntake = Intake.getInstance();
   private Limelight mLime = Limelight.getInstance();
   private Climber mClimb = Climber.getInstance();
   private XboxController xbox = new XboxController(0);
   private Joystick mDriverJoystick = new Joystick(1);


/**forward movement axis*/ 
   private double fwd;

/**strafing movement axis*/ 
   private double str;
  
/**rotational movement axis*/ 
   private double rcw;

   private Command m_autonomousCommand;
   private SendableChooser<AutoPosition> positionChooser = new SendableChooser<>();
   private ShuffleboardTab mTab = Shuffleboard.getTab("Match");
   private ComplexWidget positionChooserTab = mTab.add("Auto Chooser", positionChooser).withWidget(BuiltInWidgets.kSplitButtonChooser);
   
   private SendableChooser<veloch> veloshufflething = new SendableChooser<veloch>();
   
/**for wpi swerve and auto*/
   private RobotContainer m_robotContainer;
   
/**states of autonomous*/ 
   public enum AutoPosition {
     LEFT, NOTHING
   }

/**states of shooter speed*/
   public enum veloch
   {
     veloOne, veloTwo, veloThree
   }

/**for slider widget*/
   private double slider;
   private double driveSlider;

/**for motor volt testing*/
   private double volts;

/**baby toggle fast*/
   private boolean toggleFastOn = false;

/**baby pressed fast*/
   private boolean toggleFastPressed = false;

/**baby toggle slow*/
   private boolean toggleSlowOn = false;

/**baby pressed slow*/
   private boolean toggleSlowPressed = false;

/**constant that divides speed (baby control)*/
   private double spee = 0;

/**setpoint variable for shooting*/
   private int velo = 0;

   private double ffcalc = 0;

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
     //etc.
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
         m_autonomousCommand = new DriveStr8();//m_robotContainer.getAutonomousCommand();
         break;
       case NOTHING: // might break idk prob not
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
     //// need to see if drive initiated with code runned, problem if not runned
     //it runs everything except encoder reset i guess, but also need to set offsets back negative see if work
     Shuffleboard.addEventMarker("Teleop Init", EventImportance.kNormal);
     if (m_autonomousCommand != null) {
       m_autonomousCommand.cancel();
     }
     mDrive.resetDrive();
     SmartDashboard.putNumber("slider", 0);
     SmartDashboard.putNumber("driveSlider", 0);
     switch (veloshufflething.getSelected()) {
      case veloOne:
        velo = 0;
        break;
        
      case veloTwo:
        velo = 2000;
        break;
       
      case veloThree:
        velo = 4000;
        break;
    }
    SmartDashboard.putNumber("velo", velo);
    //TODO do i even need velocity control in another tab?
   }
 
 
  @Override
  public void teleopPeriodic() {
    
    mDrive.driveUpdate();
    mShoot.shooterUpdate();
    mIntake.updateIntake();
    
    //!are toggle functions using lots of cpu / ram? idk. hope it isnt causing a problem
    updateFastToggle();
    updateSlowToggle();

    //MouseInfo.getPointerInfo();
    //TODO do something with mouse?

    fwd = (xbox.getRawAxis(BUTTONS.forwardAxis) - DRIVE.deadband) / (1 - DRIVE.deadband);
    str = (xbox.getRawAxis(BUTTONS.strafeAxis) - DRIVE.deadband) / (1 - DRIVE.deadband);
    rcw = (xbox.getRawAxis(BUTTONS.spinAxis) - TURN.deadband) / (1 - TURN.deadband);
      


      if(Math.abs(xbox.getRawAxis(BUTTONS.forwardAxis)) < 0.1)
      {
        fwd = 0;
      }
      if(Math.abs(xbox.getRawAxis(BUTTONS.strafeAxis)) < 0.1)
      {
        str = 0;
      }
      if(Math.abs(xbox.getRawAxis(BUTTONS.spinAxis)) < 0.1)
      {
        rcw = 0;
      }

      

      if(fwd != 0 || str != 0 || rcw != 0)
      {
        //weird negative cuz robot is weird. should be negative fwd positive str rcw, 
        //but cuz of weird reasons i dont know of its just like this so yeah
        mDrive.etherSwerve(fwd/spee,-str/spee,rcw/spee); //+,-,+
        //mDrive.updateDriveDriveRaw();
      }
      else if(xbox.getAButton())
      {
        mDrive.turnCalcPercent(0, 0, 0, 0);
      }
      else if(mDriverJoystick.getRawButton(BUTTONS.driveHundredButton))
      {
        //mDrive.drivePercent(1, 1, 1, 1);
       // mDrive.driveVelocity(driveSlider, driveSlider, driveSlider, driveSlider);
        //mDrive.updateDriveDriveRaw();
      }
      else
      {
        mDrive.turnPercent(0,0,0,0);
        mDrive.drivePercent(0,0,0,0);
      }

      if(xbox.getBButton())
      {
        mDrive.resetDrive();
      }
      if(xbox.getXButton())
      {
        mDrive.resetNavx();
      }
      if(xbox.getYButton())
      {
        mDrive.encoderZero();
      }



      if(mDriverJoystick.getRawButton(BUTTONS.shooterButton))
      {
        ffcalc = -mShoot.shooterFeedForward(slider) + slider;
        mShoot.setShooterNativeVeloctiy(ffcalc);
        //mShoot.setShooterPercent(xbox.getRawAxis(2));
      }
      else if(xbox.getStartButton())
      {
        mShoot.setShooterPercent(1);
      }
      else
      {
        mShoot.setShooterPercent(0);
      }

      if(mDriverJoystick.getRawButtonPressed(BUTTONS.limelightButton))
      {
        mLime.limelightToggle();
      }

      if(mDriverJoystick.getRawButton(BUTTONS.elevatorForwardButton))
      {
        mElevator.setElevatorPercent(-1);
      }
      else
      {
        mElevator.setElevatorPercent(0);
      }



      if(mDriverJoystick.getRawButton(BUTTONS.intakeup))
      {
        //mIntake.setRollersPercent(0.5);
        mIntake.setIntakePercent(.5);
      }
      else if(mDriverJoystick.getRawButton(BUTTONS.intakedown))
      {
        mIntake.setIntakePercent(-.5);
      }
      else
      {
        //mIntake.setRollersPercent(0);
        mIntake.setIntakePercent(0);
      }



      if(mDriverJoystick.getRawButton(BUTTONS.rollersForwardButton))
      {
        mIntake.setRollersPercent(1);
      }
      else if(mDriverJoystick.getRawButton(BUTTONS.rollersBackwardButton))
      {
        mIntake.setRollersPercent(-1);
      }
      else
      {
        mIntake.setRollersPercent(0);
      }


      if(xbox.getRawButton(BUTTONS.climbUpButton))
      {
        mClimb.telescopePercent(0.5, 0.5);
      }
      else if(xbox.getRawButton(BUTTONS.climbDownButton))
      {
        mClimb.telescopePercent(-0.5, -0.5);
      }
      else
      {
        mClimb.telescopePercent(0, 0);
      }

     /* if(mDriverJoystick.getRawButton(7))
      {
        mIntake.setIntakePercent(1);
      }
      else
      {
        mIntake.setIntakePercent(0);
      }
*/

//// had them as tog fast = 1 and tog slow = 7, see if switching them made it better since it didnt work before
//works
      if(toggleFastOn){
        // Do something when toggled on
        spee = 7;
      }
      else if(toggleSlowOn){
        spee = 1;
      }
      else{
          // Do something when toggled off
        spee = 3;
      }

      //SmartDashboard.putNumber("x", MkUtil.metersToInches(mOdo.getX()));
      //SmartDashboard.putNumber("y",  MkUtil.metersToInches(mOdo.getY()));
 
      slider = SmartDashboard.getNumber("slider", 0);
      driveSlider = SmartDashboard.getNumber("driveSlider", 0);
      SmartDashboard.putNumber("spee", spee);
      //SmartDashboard.putNumber("feedf", mShoot.shooterFeedForward(slider));
      //SmartDashboard.putNumber("ffcalc", ffcalc);
    }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {
  }

/**updates state of fast toggle for driving */
  public void updateFastToggle() {
    if (xbox.getPOV() == BUTTONS.babySpeedFastAngle) {
      if (!toggleFastPressed) {
        toggleFastOn = !toggleFastOn;
        toggleSlowOn = false;
        //// ^^ might not work
        //works
        toggleFastPressed = true;
      }
    } else {
      toggleFastPressed = false;
    }
  }

/**updates state of slow toggle for driving*/
  public void updateSlowToggle()
  {
      if(xbox.getPOV() == BUTTONS.babySpeedToddlerAngle){
          if(!toggleSlowPressed){
              toggleSlowOn = !toggleSlowOn;
              toggleFastOn = false;
              //// ^^ might not work
              //works
              toggleSlowPressed = true;
          }
      }
      else{
          toggleSlowPressed = false;
      }
  }
}
