// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commandments;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Elevator;
import frc.robot.Intake;
import frc.robot.Shooter;
import frc.robot.Constants.SHOOT;

public class Shoot extends CommandBase {
  /** Creates a new Elevator. */
  private double eleSetpoint;
  private double rollerSetpoint;
  private double shootSetpoint;
  private Elevator mElevator = Elevator.getInstance();
  private Shooter mShoot = Shooter.getInstance();
  private Intake mIntake = Intake.getInstance();
  public Shoot(double eleSetpoint, double rollerSetpoint, double shootSetpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.eleSetpoint = eleSetpoint;
    this.rollerSetpoint = rollerSetpoint;
    this.shootSetpoint = shootSetpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mShoot.setShooterNativeVeloctiy(SHOOT.wackyShooterVelocity);
    if((mShoot.getShootLeftVelocity() + mShoot.getShootRightVelocity())/2 < SHOOT.wackyShooterVelocity)
    {
      mElevator.setElevatorPercent(eleSetpoint);
      mIntake.setRollersPercent(rollerSetpoint);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mElevator.setElevatorPercent(0);
    mShoot.setShooterPercent(0);
    mIntake.setRollersPercent(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
