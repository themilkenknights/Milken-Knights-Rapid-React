// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AUTO;

@SuppressWarnings("PMD.ExcessiveImports")
public class DriveSubsystem extends SubsystemBase {
  // Robot swerve modules
  private final SwerveModule m_frontLeft =
      new SwerveModule(
          Drive.getInstance().topDriveLeft,
          Drive.getInstance().topTurnLeft,
          Drive.getInstance().topTurnLeftEncoder);
  private final SwerveModule m_rearLeft =
      new SwerveModule(
        Drive.getInstance().topDriveRight,
        Drive.getInstance().topTurnRight,
        Drive.getInstance().topTurnRightEncoder);

  private final SwerveModule m_frontRight =
      new SwerveModule(
        Drive.getInstance().bottomDriveLeft,
        Drive.getInstance().bottomTurnLeft,
        Drive.getInstance().bottomTurnLeftEncoder);

  private final SwerveModule m_rearRight =
      new SwerveModule(
        Drive.getInstance().bottomDriveRight,
        Drive.getInstance().bottomTurnRight,
        Drive.getInstance().bottomTurnRightEncoder);

  // The gyro sensor
  private final Gyro m_gyro = Drive.getInstance().navX;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(AUTO.kDriveKinematics, m_gyro.getRotation2d());

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {}

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        new Rotation2d(getHeading()),
        m_frontLeft.getState(),
        m_rearLeft.getState(),
        m_frontRight.getState(),
        m_rearRight.getState());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        AUTO.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, AUTO.maxDriveVelo);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, AUTO.maxDriveVelo);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }
/*
  
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }
*/
  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate();
  }

  public void resetPID()
  {
    m_frontLeft.resetPID();
    m_frontRight.resetPID();
    m_rearLeft.resetPID();
    m_rearRight.resetPID();
  }
}