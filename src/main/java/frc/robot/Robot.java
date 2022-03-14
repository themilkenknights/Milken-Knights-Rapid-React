// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



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
import frc.robot.Commands.Commandments.DriveStraightREAL;
import frc.robot.Constants.BUTTONS;
import frc.robot.Constants.DRIVE;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.SHOOT;
import frc.robot.Constants.TURN;
import frc.robot.Drive.ETHERAUTO;
import frc.robot.Drive.ETHERRCW;
import frc.robot.WPI.RobotContainer;
import frc.robot.miscellaneous.Lights;
import frc.robot.miscellaneous.Shuffle;
import frc.robot.miscellaneous.CommandArray;

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
   private Lights mLights = Lights.getInstance();
   private Hood mHood = Hood.getInstance();
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

   private CommandArray testCommandArray = new CommandArray("testCommandArray");
   
   
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

   private boolean toggleClimbOn = false;
   private boolean toggleClimbPressed = false;

/**constant that divides speed (baby control)*/
   private double spee = 0;

/**setpoint variable for shooting*/
   private int velo = 0;

   private double ffcalc = 0;

   private boolean leftGoingUp = false;
   private boolean rightGoingUp = false;

   private boolean toggleLeftClimbOn = false;
   private boolean toggleRightClimbOn = false;

   private double inches = 21;
   private double RCW = 0;
   private double maxVelo = DRIVE.magicVelo;
   private double maxAccel = DRIVE.magicAccel;
   private ETHERAUTO mode = ETHERAUTO.Straight;
   private ETHERRCW turny = ETHERRCW.Specific;
   private double angle = 90;
   private double turnyAngle = 90;

   @Override
   public void robotInit() {
     testCommandArray.addParallelCommandGroup(new DriveStraightREAL(inches, RCW, maxVelo, maxAccel, mode, turny, angle, turnyAngle).withTimeout(6));
     //Shuffleboard.startRecording();
     m_robotContainer = new RobotContainer();
     mTab.add("velochoose", veloshufflething).withWidget(BuiltInWidgets.kSplitButtonChooser);
     mTab.add("davx",mDrive.getNavx());
     Shuffleboard.selectTab("Match");
     positionChooser.addOption("Nothing", AutoPosition.NOTHING);
     positionChooser.setDefaultOption("Left Trench", AutoPosition.LEFT);
     veloshufflething.addOption("spee1", veloch.veloOne);
     veloshufflething.addOption("spee2", veloch.veloTwo);
     veloshufflething.addOption("spee3", veloch.veloThree);
     veloshufflething.setDefaultOption("spee1", veloch.veloOne);
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
     mDrive.encoderZero();
     mDrive.resetDrive();
     mDrive.resetNavx();
     //TODO if zeroyaw in reset navx dont work, remove this and add it into a auto function
     switch (positionChooser.getSelected()) {
       case LEFT:
         m_autonomousCommand = testCommandArray.asSequentialCommandGroup();//new DriveStr8();//m_robotContainer.getAutonomousCommand();
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
     toggleClimbOn = false;
     toggleClimbPressed = false;
     toggleFastOn = false;
     toggleFastPressed = false;
     toggleSlowOn = false;
     toggleSlowPressed = false;
     toggleLeftClimbOn = false;
     toggleRightClimbOn = false;
     spee = 3;
     leftGoingUp = false;
     rightGoingUp = false;
     mDrive.encoderZero();
     //mClimb.zeroVClimbb();
      Shuffleboard.addEventMarker("Teleop Init", EventImportance.kNormal);
     if (m_autonomousCommand != null) {
       m_autonomousCommand.cancel();
     }
     mDrive.resetDrive();
     SmartDashboard.putNumber("slider", 0);
     //SmartDashboard.putNumber("driveSlider", 0);
     switch (veloshufflething.getSelected()) {
      case veloOne:
        velo = 0;
        break;
        
      case veloTwo:
        velo = 2000;
        break;
       
      case veloThree:
        velo = 4000;
        break;}
    }
 
  @Override
  public void teleopPeriodic() {
    
    mDrive.driveUpdate();
    //mShoot.shooterUpdate();
    //mIntake.updateIntake();
    mClimb.climberUpdate();
    
    //!are toggle functions using lots of cpu / ram? idk. hope it isnt causing a problem
    updateFastToggle();
    updateSlowToggle();
   updateClimbToggle();
//i <3 MILFZ and Steven Hawking 
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
        //weird negative cuz robot is weird. should be negative fwd positive str rcw
        mDrive.etherSwerve(fwd/spee,-str/spee,rcw/spee); //+,-,+
        //mDrive.updateDriveDriveRaw();
      }
      else if(xbox.getAButton())
      {
        mDrive.turnCalcPercent(0, 0, 0, 0);
      }
      /*else if(mDriverJoystick.getRawButton(BUTTONS.driveHundredButton))
      {
        //mDrive.drivePercent(1, 1, 1, 1);
       // mDrive.driveVelocity(driveSlider, driveSlider, driveSlider, driveSlider);
        //mDrive.updateDriveDriveRaw();
      }*/
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
        ffcalc = -mShoot.shooterFeedForward(SHOOT.wackyShooterVelocity) + SHOOT.wackyShooterVelocity;
        mShoot.setShooterNativeVeloctiy(ffcalc);
        if((mShoot.shootLeft.getSelectedSensorVelocity() + mShoot.shootRight.getSelectedSensorVelocity())/2 >= SHOOT.wackyShooterVelocity)
        {
          mElevator.setElevatorPercent(ELEVATOR.mySpeed);
          mIntake.setRollersPercent(1);
        }
        //mShoot.setShooterPercent(xbox.getRawAxis(2));
      }
      else if(xbox.getStartButton())
      {
        ffcalc = -mShoot.shooterFeedForward(slider) + slider;
        mShoot.setShooterNativeVeloctiy(ffcalc);
      }
      else
      {
        mShoot.setShooterPercent(0);
      }

      /*if(mDriverJoystick.getRawButtonPressed(BUTTONS.limelightButton))
    {
        mLime.limelightToggle();
      }
*/
      if(mDriverJoystick.getRawButton(BUTTONS.elevatorForwardButton))
      {
        mElevator.setElevatorPercent(-.8);
      }
      else if(mDriverJoystick.getRawButton(BUTTONS.elevatorBackwardButton))
{
      mElevator.setElevatorPercent(.8);
      }
      /*else
      {
        mElevator.setElevatorPercent(0);
      }*/

      


      if(mDriverJoystick.getRawButton(BUTTONS.intakeup))
      {
        //mIntake.setRollersPercent(0.5);
        mIntake.setIntakePercent(.9);
      }
      else if(mDriverJoystick.getRawButton(BUTTONS.intakedown))
      {
        mIntake.setIntakePercent(-.9);
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
      /*else
      {
        mIntake.setRollersPercent(0);
      }*/
 





    

/*
    if(toggleClimbOn == false)
    {
      if(mDriverJoystick.getRawButton(BUTTONS.climbUpButton))
      {
        mClimb.telescopePercent(0.5, -0.5);
      }
      else if(mDriverJoystick.getRawButton(BUTTONS.climbDownButton))
      {
        mClimb.telescopePercent(-0.5, 0.5);
      }
      else
      {
        mClimb.telescopePercent(0, 0);
      }
    }

    else
    {
      if(xbox.getRawButton(BUTTONS.climbRightUpButton))
      {
        mClimb.telescopePercentRight(0.5);
      }
      else if(xbox.getRawAxis(BUTTONS.climbRightDownAxis) > 0.2)
      {
        mClimb.telescopePercentRight(-0.5);
      }
      else
      {
        mClimb.telescopePercentRight(0);
      }



      if(xbox.getRawButton(BUTTONS.climbLeftUpButton))
      {
        mClimb.telescopePercentLeft(-0.5);
      }
      else if(xbox.getRawAxis(BUTTONS.climbLeftDownAxis) > 0.2)
      {
        mClimb.telescopePercentLeft(0.5);
      }
      else
      {
        mClimb.telescopePercentLeft(0);
      }
    }
*/


if(mDriverJoystick.getPOV() == BUTTONS.climbUpAxis)
{
  mClimb.telescopePercent(1, -1);
}
else if(mDriverJoystick.getPOV() == BUTTONS.climbDownAxis)
{
  mClimb.telescopePercent(-1, -1);
}

if(mDriverJoystick.getRawButton(BUTTONS.climbRightUpButton) && mClimb.isRightBelow())
{
  mClimb.telescopePercentRight(1);
}
else if(mDriverJoystick.getRawButton(BUTTONS.climbRightDownButton) && mClimb.isRightAbove())
{
  mClimb.telescopePercentRight(-1);
}



if(mDriverJoystick.getRawButton(BUTTONS.climbLeftUpButton) && mClimb.isLeftBelow())
{
  mClimb.telescopePercentLeft(1);
}
else if(mDriverJoystick.getRawButton(BUTTONS.climbLeftDownButton) && mClimb.isLeftAbove())
{
  mClimb.telescopePercentLeft(-1);
}




if(!mClimb.isLeftAbove() && !leftGoingUp)
{
  toggleLeftClimbOn = false;
  mClimb.zeroLeftClimb();
}

if(!mClimb.isRightAbove() && !rightGoingUp)
{
  toggleRightClimbOn = false;
  mClimb.zeroRightClimb();
}

if(!mClimb.isLeftBelow() && leftGoingUp)
{
  toggleLeftClimbOn = false;
}

if(!mClimb.isRightBelow() && rightGoingUp)
{
  toggleRightClimbOn = false;
}


if((!mClimb.isLeftAbove() && !leftGoingUp) && (!mClimb.isRightAbove() && !rightGoingUp)) 
{
  leftGoingUp = true;
  rightGoingUp = true;
}

if((!mClimb.isLeftBelow() && leftGoingUp) && (!mClimb.isRightBelow() && rightGoingUp))
{
  leftGoingUp = false;
  rightGoingUp = false;
}

if(!(mDriverJoystick.getPOV() == BUTTONS.climbUpAxis) &&
!(mDriverJoystick.getPOV() == BUTTONS.climbDownAxis) &&
!mDriverJoystick.getRawButton(BUTTONS.climbLeftDownButton) &&
!mDriverJoystick.getRawButton(BUTTONS.climbLeftUpButton)&&
!toggleLeftClimbOn)
{
  mClimb.telescopePercentLeft(0);
}

if(!(mDriverJoystick.getPOV() == BUTTONS.climbUpAxis) &&
!(mDriverJoystick.getPOV() == BUTTONS.climbDownAxis) &&
!mDriverJoystick.getRawButton(BUTTONS.climbRightDownButton) &&
!mDriverJoystick.getRawButton(BUTTONS.climbRightUpButton) &&
!toggleRightClimbOn)
{
  mClimb.telescopePercentRight(0);
}

if((mDriverJoystick.getPOV() == BUTTONS.climbUpAxis) ||
(mDriverJoystick.getPOV() == BUTTONS.climbDownAxis) ||
mDriverJoystick.getRawButton(BUTTONS.climbLeftDownButton) ||
mDriverJoystick.getRawButton(BUTTONS.climbLeftUpButton) ||
mDriverJoystick.getRawButton(BUTTONS.climbRightDownButton) ||
mDriverJoystick.getRawButton(BUTTONS.climbRightUpButton))
{
  toggleLeftClimbOn = false;
  toggleRightClimbOn = false;
}


if((mShoot.getShootRightVelocity() + mShoot.getShootLeftVelocity())/2 < SHOOT.wackyShooterVelocity &&
!(mDriverJoystick.getRawButton(BUTTONS.rollersForwardButton)) &&
!(mDriverJoystick.getRawButton(BUTTONS.rollersBackwardButton)) &&
!(mDriverJoystick.getRawButton(BUTTONS.elevatorBackwardButton)) &&
!(mDriverJoystick.getRawButton(BUTTONS.elevatorForwardButton)))
{
  mIntake.setRollersPercent(0);
  mElevator.setElevatorPercent(0);
}


if(toggleLeftClimbOn)
{
  mClimb.climbAutoLeft(leftGoingUp);
}

if(toggleRightClimbOn)
{
  mClimb.climbAutoRight(rightGoingUp);
}


/*

isLeftAbove = true
isLeftBelow = false

-
|
|       isLeftAbove = true
|       isLeftBelow = true
|
-
isLeftAbove = false
isLeftBelow = true
*/


      if(toggleFastOn){
        // Do something when toggled on
        spee = 7;
      }
      else if(toggleSlowOn){
        spee = 1;
        mLights.french();
      }
      else{
          // Do something when toggled off
        spee = 3;
      }

      //SmartDashboard.putNumber("x", MkUtil.metersToInches(mOdo.getX()));
      //SmartDashboard.putNumber("y",  MkUtil.metersToInches(mOdo.getY()));
 
      slider = SmartDashboard.getNumber("slider", 0);
      //driveSlider = SmartDashboard.getNumber("driveSlider", 0);
      SmartDashboard.putNumber("spee", spee);
      SmartDashboard.putBoolean("right on", toggleRightClimbOn);
      SmartDashboard.putBoolean("left on", toggleLeftClimbOn);
      //SmartDashboard.putNumber("povjoy", mDriverJoystick.getPOV());
      //SmartDashboard.putNumber("povx", xbox.getPOV());
      //SmartDashboard.putNumber("feedf", mShoot.shooterFeedForward(slider));
      //SmartDashboard.putNumber("ffcalc", ffcalc);

      //SmartDashboard.putNumber("test xbox", (xbox.getRawAxis(BUTTONS.forwardAxis) * Math.cos(Math.toRadians(0))) + (xbox.getRawAxis(BUTTONS.strafeAxis) *  Math.sin(Math.toRadians(0))));
      //SmartDashboard.putNumber("test box two", Math.atan2(xbox.getRawAxis(BUTTONS.forwardAxis),xbox.getRawAxis(BUTTONS.strafeAxis))*180/Constants.kPi);
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
       
              toggleSlowPressed = true;
          }
      }
      else{
          toggleSlowPressed = false;
      }
  }


  public void updateClimbToggle()
  {
      if(mDriverJoystick.getRawButton(BUTTONS.climbAutoButton)){
          if(!toggleClimbPressed){
              toggleLeftClimbOn = !toggleLeftClimbOn;
              toggleRightClimbOn = !toggleRightClimbOn;

              
              toggleClimbPressed = true;
          }
      }
      else{
          toggleClimbPressed = false;
      }
  }
  
}
