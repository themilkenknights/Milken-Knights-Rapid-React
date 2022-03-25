// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Drive;
import frc.robot.Commandments.DriveStraight;
import frc.robot.Commandments.DriveStraightREALSUPERREAL;
import frc.robot.Commandments.Shoot;
import frc.robot.Commandments.Turn;
import frc.robot.Constants.DRIVE;
import frc.robot.Constants.SHOOT;
import frc.robot.Constants.TURN;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class JacksAuto extends SequentialCommandGroup {
  private Drive mDrive = Drive.getInstance();
  private double distanceA = -120;
  private double lengthB = -200;
  private double angle = mDrive.calculateAngleOfPath(distanceA, lengthB);
  private double maxAccel = 15000; 
  private double distanceA2 = -220;
  private double lengthB2 = -300;
  private double angle2 = mDrive.calculateAngleOfPath(distanceA2, lengthB2);

/**Curved drive sequential command*/
  public JacksAuto() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());


    addCommands(deadline(new Shoot(-0.7, 0.5, SHOOT.jacksWackyShooterVelocity).withTimeout(3)), 
               deadline(new DriveStraight(distanceA, lengthB, 1, -((angle) % 90), DRIVE.magicVelo, DRIVE.magicAccel).withTimeout(6)),
              //!deadline(new Turn(0).withTimeout(1)), dont know if we need these
              deadline(new DriveStraight(distanceA2, lengthB2, 1, -((angle2) % 90), DRIVE.magicVelo, DRIVE.magicAccel).withTimeout(6)),

              //strafe
            deadline(new DriveStraightREALSUPERREAL(20, 90, DRIVE.magicVelo, DRIVE.magicAccel).withTimeout(6)));
            
      //addCommands(deadline(new DriveStraightREAL(10, 1, 0)));
  }
}
