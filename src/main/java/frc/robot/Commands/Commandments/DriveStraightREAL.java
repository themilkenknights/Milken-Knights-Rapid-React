// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Commandments;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Drive;
import frc.robot.Drive.ETHERAUTO;
import frc.robot.Drive.ETHERRCW;
//TODO hopefully this works
public class DriveStraightREAL extends CommandBase {
  private double inches;
  private double RCW;
  private double maxVelo;
  private double maxAccel;
  private double lengthB;
  private double angle;//mDrive.calculateAngleOfPath(distanceA, lengthB);
  private ETHERAUTO mode;
  private ETHERRCW turny;
  private double turnyAngle;
  Drive mDrive = Drive.getInstance();
  /**
   * a true drive straight command for autonomous (uses motion magic for drive and ether for turn)
   * @param inches how far you want to go
   * @param FWD y value as if it was from the xbox controller [-1, 1]
   * @param STR x value as if it was from the xbox controller [-1, 1]
   */
  public DriveStraightREAL(double inches, double RCW, double maxVelo, double maxAccel, ETHERAUTO mode, ETHERRCW turny, double angle, double turnyAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.inches = inches;
    this.maxVelo = maxVelo;
    this.maxAccel = maxAccel;
    this.RCW = RCW;
    this.lengthB = 0;
    this.angle = angle;
    this.mode = mode;
    this.turny = turny;
    this.turnyAngle = turnyAngle;
  }
  
  public DriveStraightREAL(double inches, double lengthB, double RCW, double maxVelo, double maxAccel, ETHERAUTO mode, ETHERRCW turny, double turnyAngle)
  {
    this.turnyAngle = turnyAngle;
    this.inches = inches;
    this.lengthB = lengthB;
    this.maxVelo = maxVelo;
    this.maxAccel = maxAccel;
    this.RCW = RCW;
    this.turny = turny;
    this.mode = mode;
    this.angle = -((mDrive.calculateAngleOfPath(inches, lengthB) % 90));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //mDrive.setMagicStraight(inches, maxVelo, maxAccel);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   // mDrive.updateMagicStraight();
    mDrive.autoTurnUpdate(inches, angle, RCW, mode, turny, turnyAngle); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mDrive.isEtherMoveDone(inches);
  }
}
