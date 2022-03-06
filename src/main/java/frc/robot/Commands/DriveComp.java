// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Drive;
import frc.robot.Commands.Commandments.DriveStraightREALSUPERREAL;
import frc.robot.Commands.Commandments.Turn;
import frc.robot.Constants.DRIVE;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveComp extends SequentialCommandGroup {
  private Drive mDrive = Drive.getInstance();
  private double distanceA = -120;
  private double angle = 0;


  public DriveComp() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    //// still need to see if angle is either 90 - angle and 90 + angle or -angle and angle
    //// determining if its a unit circle 0 ordeal or a top y axis = 0 ordeal
    //sticking with wpi swervecommandcontroller for now, if that doesnt work then try and fix homemade auto

    addCommands(deadline(new Turn(0).withTimeout(1)), 
                deadline(new DriveStraightREALSUPERREAL(distanceA, angle, DRIVE.magicVelo, DRIVE.magicAccel).withTimeout(6)));

                //TODO see if above still works, then do this
      //addCommands(deadline(new DriveStraightREAL(10, 1, 0)));
  }
}
