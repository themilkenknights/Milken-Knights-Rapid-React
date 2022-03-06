// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Commandments;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Drive;

public class DriveStraightREALSUPERREAL extends CommandBase {
  private double inches;
  private double angle;
  private double maxVelo;
  private double maxAccel;

  private Drive mDrive = Drive.getInstance();
  /**
   * a truly true drive straight command for autonomous (motion magic for drive and PID for turn)
   * @param inches how far you want to go
   * @param angle at what constant angle should the turn motors be at
   */
  public DriveStraightREALSUPERREAL(double inches, double angle, double maxVelo, double maxAccel) {
    this.inches = inches;
    this.angle = angle;
    this.maxVelo = maxVelo;
    this.maxAccel = maxAccel;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mDrive.resetDrive();
    mDrive.setMagicStraight(inches, maxVelo, maxAccel);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mDrive.updateMagicStraight();
    mDrive.turnCalcPercent(angle, angle, angle, angle);
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
