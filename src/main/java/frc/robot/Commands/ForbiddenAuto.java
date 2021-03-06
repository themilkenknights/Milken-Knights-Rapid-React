// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commandments.DriveStraightREAL;
import frc.robot.Commandments.DriveStraightREALSUPERREAL;
import frc.robot.Commandments.Shoot;
import frc.robot.Commandments.Turn;
import frc.robot.Constants.DRIVE;
import frc.robot.Constants.SHOOT;
import frc.robot.Drive.ETHERAUTO;
import frc.robot.Drive.ETHERRCW;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ForbiddenAuto extends SequentialCommandGroup {
  /** Creates a new ForbiddenAuto. */



  private double inches  = -21;
  private double RCW = 0;
  private double maxVelo = DRIVE.magicVelo;
  private double maxAccel = DRIVE.magicAccel;
  private ETHERAUTO mode = ETHERAUTO.Straight;
  private ETHERRCW turny = ETHERRCW.Specific;
  private double angle = 0;
  private double turnyAngle = 0;
  public ForbiddenAuto() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
                deadline(new Shoot(1.5,-0.5, 0.5, -SHOOT.wackyShooterVelocity).withTimeout(1.5)),
                deadline(new Turn(0).withTimeout(1)), 
                deadline(new DriveStraightREAL(inches, RCW, maxVelo, maxAccel, mode, turny, angle, turnyAngle).withTimeout(10)));
  }
}
