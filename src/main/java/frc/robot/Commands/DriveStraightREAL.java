// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Drive;
//TODO hopefully this works
public class DriveStraightREAL extends CommandBase {
  /** Creates a new DriveStraightREAL. */
  private double inches;
  private int FWD;
  private int STR;
  Drive mDrive = Drive.getInstance();
  public DriveStraightREAL(double inches, int FWD, int STR) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.inches = inches;
    this.FWD = FWD;
    this.STR = STR;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mDrive.setMagicStraight(inches);
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
