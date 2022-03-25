// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Drive;
import frc.robot.Commandments.DriveStraightREALSUPERREAL;
import frc.robot.Commandments.IntakeBall;
import frc.robot.Commandments.Shoot;
import frc.robot.Commandments.Turn;
import frc.robot.Constants.DRIVE;
import frc.robot.Constants.SHOOT;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveComp extends SequentialCommandGroup {
  private Drive mDrive = Drive.getInstance();
  private double distanceA = 100;
  private double angle = 0;
  private double maxAccel = 5000;


  public DriveComp() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

 
    addCommands(//deadline(new Shoot(-0.5, 0.5, -SHOOT.wackyShooterVelocity).withTimeout(5)),

                deadline(new Turn(0).withTimeout(1)), 
                deadline(new Shoot(5,-0.5, -0.5, -SHOOT.wackyShooterVelocity).withTimeout(5)),
                deadline(new DriveStraightREALSUPERREAL(distanceA, angle, DRIVE.magicVelo, maxAccel).withTimeout(3)));
      //addCommands(deadline(new DriveStraightREAL(10, 1, 0)));
  }
}
