// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commandments;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Intake;
import frc.robot.Constants.INTAKE;
import frc.robot.miscellaneous.MkTimerV2;

public class IntakeBall extends CommandBase {
  /** Creates a new Intake. */
  private double time;
  private Intake mIntake = Intake.getInstance();
  private MkTimerV2 mTime = new MkTimerV2();
  public IntakeBall(double time) {
    this.time = time;
    mTime.setFinishTime(this.time);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mTime.startTimer(this.time);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mIntake.setRollersPercent(-INTAKE.rollerSpeed);
    mIntake.setIntakePosition(INTAKE.intakeRotationsNative);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mIntake.setIntakePercent(0);
    mIntake.setRollersPercent(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mTime.isTimerDone();
  }
}
