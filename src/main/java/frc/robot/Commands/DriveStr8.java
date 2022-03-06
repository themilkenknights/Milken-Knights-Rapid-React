// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Drive;
import frc.robot.Commands.Commandments.DriveStraight;
import frc.robot.Commands.Commandments.Turn;
import frc.robot.Constants.DRIVE;
import frc.robot.Constants.TURN;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveStr8 extends SequentialCommandGroup {
  private Drive mDrive = Drive.getInstance();
  private double distanceA = 34;
  private double lengthB = 24;
  private double angle = mDrive.calculateAngleOfPath(distanceA, lengthB);
  private double maxAccel = 600;

/**Curved drive sequential command*/
  public DriveStr8() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(deadline(new Turn(((angle) % 90))).withTimeout(1), 
                deadline(new DriveStraight(distanceA, lengthB, 0, -((angle) % 90), DRIVE.magicVelo, DRIVE.magicAccel)).withTimeout(6));

                //TODO see if above still works, then do this
      //addCommands(deadline(new DriveStraightREAL(10, 1, 0)));
  }
}
