// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commandments;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Drive;

public class Turn extends CommandBase {
  /**Creates a new Turn command. */
  private double angle;
  private Drive mDrive = Drive.getInstance();
  /**
   * turn angular motors to angle
   * @param angle desired angle
   */
  public Turn(double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mDrive.turnCalcPercent(angle, angle, angle, angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mDrive.percentTurnDone();
  }
}
