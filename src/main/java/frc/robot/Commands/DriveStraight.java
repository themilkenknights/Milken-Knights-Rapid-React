// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Drive;

public class DriveStraight extends CommandBase {
  private Drive mDrive = Drive.getInstance();
  double j;
  double distanceA;
  double lengthB;
  double angle;
  double RCW;
  public DriveStraight(double distance, double dista, double lengb, double angl, double rcw) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    j = distance;
    distanceA = dista;
    lengthB = lengb;
    angle = angl;
    RCW = rcw;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    mDrive.setMagicStraight(j);
    mDrive.autoTurnSet(distanceA, lengthB, j);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    mDrive.updateMagicStraight();
    mDrive.autoTurnUpdate(j, angle + mDrive.calculateAngleOfPath(distanceA, lengthB),RCW);
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
