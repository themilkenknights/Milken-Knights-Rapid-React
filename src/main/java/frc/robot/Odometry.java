// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DRIVE;
import frc.robot.Constants.TURN;

/** Add your docs here. */
public class Odometry {
    private Drive mDrive = Drive.getInstance();

   /* private TalonFX topDriveLeft = new TalonFX(DRIVE.topDriveLeftCANID);
    private TalonFX topDriveRight = new TalonFX(DRIVE.topDriveRightCANID);
    private TalonFX bottomDriveLeft = new TalonFX(DRIVE.bottomDriveLeftCANID);
    private TalonFX bottomDriveRight = new TalonFX(DRIVE.bottomDriveRightCANID);
*/
    private double heightMeters = MkUtil.inchesToMeters(Constants.height / 2);
    private double widthMeters = MkUtil.inchesToMeters(Constants.width / 2);

    // Locations for the swerve drive modules relative to the robot center.
    Translation2d m_frontLeftLocation = new Translation2d(heightMeters, widthMeters);
    Translation2d m_frontRightLocation = new Translation2d(heightMeters, -widthMeters);
    Translation2d m_backLeftLocation = new Translation2d(-heightMeters, widthMeters);
    Translation2d m_backRightLocation = new Translation2d(-heightMeters, -widthMeters);

    // Creating my kinematics object using the module locations
    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics
    (
        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
    );

    // Example chassis speeds: 1 meter per second forward, 3 meters
    // per second to the left, and rotation at 1.5 radians per second
    // counterclockwise.
    ChassisSpeeds speeds = new ChassisSpeeds(1.0, 3.0, 1.5);

    // Convert to module states
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

    // Front left module state
    SwerveModuleState frontLeft = moduleStates[0];

    // Front right module state
    SwerveModuleState frontRight = moduleStates[1];

    // Back left module state
    SwerveModuleState backLeft = moduleStates[2];

    // Back right module state
    SwerveModuleState backRight = moduleStates[3];
/*
    private final SwerveModule m_frontLeft = new SwerveModule(DRIVE.topDriveLeftCANID, TURN.topTurnLeftCANID, TURN.topTurnLeftCANCoderCANID);
    private final SwerveModule m_frontRight = new SwerveModule(DRIVE.topDriveRightCANID, TURN.topTurnRightCANID, TURN.topTurnRightCANCoderCANID);
    private final SwerveModule m_backLeft = new SwerveModule(DRIVE.bottomDriveLeftCANID, TURN.bottomTurnLeftCANID, TURN.bottomTurnLeftCANCoderCANID);
    private final SwerveModule m_backRight = new SwerveModule(DRIVE.bottomDriveRightCANID, TURN.bottomTurnRightCANID, TURN.bottomTurnRightCANCoderCANID);
*/
    private final SwerveModule m_frontLeft = new SwerveModule(Drive.getInstance().topDriveLeft, Drive.getInstance().topTurnLeft, Drive.getInstance().topTurnLeftEncoder);
    private final SwerveModule m_frontRight = new SwerveModule(Drive.getInstance().topDriveRight, Drive.getInstance().topTurnRight, Drive.getInstance().topTurnRightEncoder);
    private final SwerveModule m_backLeft = new SwerveModule(Drive.getInstance().bottomDriveLeft, Drive.getInstance().bottomTurnLeft, Drive.getInstance().bottomTurnLeftEncoder);
    private final SwerveModule m_backRight = new SwerveModule(Drive.getInstance().bottomDriveRight, Drive.getInstance().bottomTurnRight, Drive.getInstance().bottomTurnRightEncoder);
    
    Rotation2d navX2d = new Rotation2d(Math.toRadians(mDrive.getNavx()));
    
    // Creating my odometry object from the kinematics object. Here,
    // our starting pose is 5 meters along the long end of the field and in the
    // center of the field along the short end, facing forward.
    SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics,
    navX2d, new Pose2d(0, 0, new Rotation2d()));

    Pose2d m_pose = new Pose2d();

    public Odometry()
    {

    }

    public static Odometry getInstance()
    {
        return InstanceHolder.mInstance;
    }

    public void updateOdo()
    {
        navX2d = Rotation2d.fromDegrees(mDrive.getNavx());
            // Update the pose
            
        m_pose = m_odometry.update(
            navX2d,
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_backLeft.getState(),
            m_backRight.getState());
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        var swerveModuleStates =
            m_kinematics.toSwerveModuleStates(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, navX2d)
                    : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MkUtil.nativePer100MsToMetersPerSec(DRIVE.maxNativeVelocity));
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);
    }

    public double getX()
    {
        return m_pose.getX();
    }

    public double getY()
    {
        return m_pose.getY();
    }

    public void resetPose()
    {
        m_odometry.resetPosition(new Pose2d(0, 0, new Rotation2d()), new Rotation2d());
    }

    private static class InstanceHolder
    {
        private static final Odometry mInstance = new Odometry();
    } 
    
}
