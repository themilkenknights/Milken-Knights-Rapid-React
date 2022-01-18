// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Drive;

public class DriveStraight extends CommandBase {
  private Drive mDrive = Drive.getInstance();
  double distanceDrive;
  double angle;
  double RCW;
  /**
   * move in a curved path
    <pre>
    .
                         
                          E
                      ~~~~~~~~~
                   ~~     +     ~~     
  /o----o\       ~~       + B     ~~       /o----o\
  |  (F) |  (2) ~~        +        ~~ (1)  |  (F) |
  \o----o/      =====================      \o----o/ 
                \     A   |   A     /    
                 \        |        /
                  \       |       /
                   \      |D     / 
                    \     |     / 
                     \  __|__  / 
                      \/  C  \/
                       \  |  /
                        \ | /
                         \|/
            
    A = distanceA
    B = lengthB
    C = angle
    D = radius
    E = circumference
    F = robot
    1 = starting position
    2 = ending position
    </pre>
   * @param distanceA 
   * @param lengthB
   * @param rcw 1 through -1 for spinny, 0 for no spinny
   * @param angle calculated angle
   */
  public DriveStraight(double distanceA, double lengthB, double rcw, double angle) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.angle = angle;
    distanceDrive = mDrive.calculateArcOfPath(distanceA, lengthB);
    RCW = rcw;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    mDrive.setMagicStraight(distanceDrive);
    mDrive.autoTurnSet();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    mDrive.updateMagicStraight();
    mDrive.autoTurnUpdate(distanceDrive, angle, RCW);
    SmartDashboard.putNumber("distance", distanceDrive);
    SmartDashboard.putNumber("ang", angle);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return mDrive.isMagicStraightDone();
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {}
}