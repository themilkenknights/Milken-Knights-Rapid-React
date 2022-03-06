// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Commandments;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Drive;
//TODO hopefully this works
public class DriveStraightREAL extends CommandBase {
  private double inches;
  private int FWD;
  private int STR;
  private double maxVelo;
  private double maxAccel;
  Drive mDrive = Drive.getInstance();
  /**
   * a true drive straight command for autonomous (uses motion magic for drive and ether for turn)
   * @param inches how far you want to go
   * @param FWD y value as if it was from the xbox controller [-1, 1]
   * @param STR x value as if it was from the xbox controller [-1, 1]
   */
  public DriveStraightREAL(double inches, int FWD, int STR, double maxVelo, double maxAccel) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.inches = inches;
    this.FWD = FWD;
    this.STR = STR;
    this.maxVelo = maxVelo;
    this.maxAccel = maxAccel;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mDrive.setMagicStraight(inches, maxVelo, maxAccel);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mDrive.updateMagicStraight();
    mDrive.swerveAutonomousEther(FWD, STR, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mDrive.isMagicStraightDone();
  }
}
