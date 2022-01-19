// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveStr8 extends SequentialCommandGroup {
  /** Creates a new DriveStr8. */
  Drive mDrive = Drive.getInstance();
  double distanceA = 34;
  double lengthB = 24;
  double angle = mDrive.calculateAngleOfPath(distanceA, lengthB);
  public DriveStr8() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    //TODO still need to see if angle is either 90 - angle and 90 + angle or -angle and angle
    //! determining if its a unit circle 0 ordeal or a top y axis = 0 ordeal

    addCommands(deadline(new Turn(((angle) % 90))).withTimeout(2), 
                deadline(new DriveStraight(distanceA, lengthB, 0, -((angle) % 90))).withTimeout(5));
  }
}
