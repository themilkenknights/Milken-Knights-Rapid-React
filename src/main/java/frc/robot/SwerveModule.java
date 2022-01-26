// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AUTO;
import frc.robot.Constants.DRIVE;
import frc.robot.Constants.TURN;

public class SwerveModule {
  private static final double kWheelRadius = MkUtil.inchesToMeters(DRIVE.kWheelDiameterInches/2); //0.0508;
  private static final int kEncoderResolution = 2048;

  private static final double kModuleMaxAngularVelocity = AUTO.maxModuleTurnVelo;
  private static final double kModuleMaxAngularAcceleration = AUTO.maxModuleTurnAccel;

  private final TalonFX m_driveMotor;
  private final TalonFX m_turningMotor;

  private final CANCoder m_turningEncoder;

  // Gains are for example purposes only - must be determined for your own robot!
  //private final PIDController m_drivePIDController = new PIDController(AUTO.moduleDriveKP, AUTO.moduleDriveKI, AUTO.moduleDriveKD);

  // Gains are for example purposes only - must be determined for your own robot!
  /*private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          AUTO.moduleTurnKP,
          AUTO.moduleTurnKI,
          AUTO.moduleTurnKD,
          new TrapezoidProfile.Constraints(
              kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));*/

  // Gains are for example purposes only - must be determined for your own robot!
  //private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(DRIVE.kS, DRIVE.kV);
  //private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(TURN.kS, TURN.kV);

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotorChannel PWM output for the drive motor.
   * @param turningMotorChannel PWM output for the turning motor.
   * @param turningEncoderChannel DIO input for the turning encoder channel B
   */
  public SwerveModule(
      TalonFX driveMotorChannel,
      TalonFX turningMotorChannel,
      CANCoder turningEncoderChannel) {
    m_driveMotor = driveMotorChannel;
    m_turningMotor = turningMotorChannel;

    m_turningEncoder = turningEncoderChannel;

    //TODO test kp first, then see if using p gets us setpoint in time on ground. if ocilation, add d. when it works on ground, add f 
    //TODO ask swerd or shalit how to choose falcon pid
    m_driveMotor.config_kP(0, AUTO.moduleDriveKP);
    m_driveMotor.config_kI(0, AUTO.moduleDriveKI);
    m_driveMotor.config_kD(0, AUTO.moduleDriveKD);
    m_driveMotor.config_kF(0, 0);

    m_turningMotor.config_kP(0, AUTO.moduleTurnKP);
    m_turningMotor.config_kI(0, AUTO.moduleTurnKI);
    m_turningMotor.config_kD(0, AUTO.moduleTurnKD);
    m_turningMotor.config_kF(0, 0);


    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    //!m_driveEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    //!m_turningEncoder.setDistancePerPulse(2 * Math.PI / kEncoderResolution);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    //m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(MkUtil.nativePer100MsToMetersPerSec(m_driveMotor.getSelectedSensorVelocity()), new Rotation2d(Math.toRadians(m_turningEncoder.getAbsolutePosition())));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(Math.toRadians(m_turningEncoder.getAbsolutePosition())));

    // Calculate the drive output from the drive PID controller.
    //!final double driveOutput =
    //!    m_drivePIDController.calculate(MkUtil.nativePer100MsToMetersPerSec(m_driveMotor.getSelectedSensorVelocity()), state.speedMetersPerSecond);

    //final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    //!final double turnOutput =
    //!    m_turningPIDController.calculate(Math.toRadians(m_turningEncoder.getAbsolutePosition()), state.angle.getRadians());

    //final double turnFeedforward =
    //    m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    m_driveMotor.set(ControlMode.Velocity, MkUtil.metersPerSecondToNativeUnitsPer100Ms(state.speedMetersPerSecond));  // + driveFeedforward);
    m_turningMotor.set(ControlMode.Position, MkUtil.degreesToNative(state.angle.getDegrees(), TURN.greerRatio));  // + turnFeedforward);

    //TODO could use integrated pid, but doesnt have continuous input. need to think about it

    SmartDashboard.putNumber("driving motor", MkUtil.nativePer100MsToMetersPerSec(m_driveMotor.getSelectedSensorVelocity()));
    SmartDashboard.putNumber("drive setpoint", state.speedMetersPerSecond);
    //!SmartDashboard.putNumber("turning motor", turnOutput);
    //!SmartDashboard.putNumber("turning feed motor", turnFeedforward);

    //SmartDashboard.putNumber("turn set", m_turningPIDController.getSetpoint().position);
    //SmartDashboard.putNumber("drive set", m_drivePIDController.getSetpoint());


    //// use cancoder for sysid testing, and multiply cancoder by ((2*Math.PI)/360) ------ nvm thats just pi/180 same thing
    //also big changes with conversion factors, redthunder7166 maybe
  }

  /**
   * resets PIDs
   * @return returns fresh PIDs
   */
  public void resetPID()
  {
    //m_drivePIDController.reset();
    //m_turningPIDController.reset(Math.toRadians(m_turningEncoder.getAbsolutePosition()));
  }
}