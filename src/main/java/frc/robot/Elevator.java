// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import frc.robot.Constants.ELEVATOR;

/**going up...*/
public class Elevator {
    TalonFX elevator = new TalonFX(ELEVATOR.elevatorCANID);
    private Elevator()
    {
        elevator.configFactoryDefault();
        elevator.setNeutralMode(NeutralMode.Coast);
        elevator.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        elevator.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_25Ms);
        elevator.configVelocityMeasurementWindow(ELEVATOR.velocityMeasAmount);
        elevator.enableVoltageCompensation(true);
        elevator.configVoltageCompSaturation(ELEVATOR.voltComp);
        elevator.config_kP(0, ELEVATOR.elevatorKP);
        elevator.config_kI(0, ELEVATOR.elevatorKI);
        elevator.config_kD(0, ELEVATOR.elevatorKD);
        elevator.config_kF(0, ELEVATOR.elevatorKF);
        elevator.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, ELEVATOR.statusOneMeas);
        elevator.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, ELEVATOR.statusTwoMeas);
    }

    public static Elevator getInstance()
    {
        return InstanceHolder.mInstance;
    }

    public void updateElevator()
    {
        SmartDashboard.putNumber("elerpm", elevator.getSelectedSensorVelocity());
    }

    public void setElevatorPercent(double setpoint)
    {
        elevator.set(ControlMode.PercentOutput, setpoint);
    }

    public void setElevatorVelocity(double setpoint)
    {
        elevator.set(ControlMode.Velocity, setpoint);
    }

    public double elevatorFeedForward(double setpoint)
    {
        return ELEVATOR.maxError * (Math.cos((Constants.kPi / 2) * (1+(setpoint / ELEVATOR.maxVelo))));
    }

    private static class InstanceHolder
    {
        private static final Elevator mInstance = new Elevator();
    } 
    
}
