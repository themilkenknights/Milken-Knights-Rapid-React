// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Commandments;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Intake;

public class Rolling extends CommandBase {
  /** Creates a new Rolling. */
  private double setpoint;
  private Intake mIntake = Intake.getInstance();
  public Rolling(double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.setpoint = setpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mIntake.setRollersPercent(setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
