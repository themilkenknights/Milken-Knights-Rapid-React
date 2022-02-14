// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Constants.ELEVATOR;

/** going up... */
public class Elevator {
    TalonSRX elevator = new TalonSRX(ELEVATOR.elevatorCANID);
    private Elevator()
    {
        elevator.configFactoryDefault();
        elevator.setNeutralMode(NeutralMode.Brake);
        elevator.setInverted(false);
        elevator.enableVoltageCompensation(true);
        elevator.configVoltageCompSaturation(ELEVATOR.voltComp);
    }

    public static Elevator getInstance()
    {
        return InstanceHolder.mInstance;
    }

    public void setElevatorPercent(double setpoint)
    {
        elevator.set(ControlMode.PercentOutput, setpoint);
    }

    private static class InstanceHolder
    {
        private static final Elevator mInstance = new Elevator();
    } 
}
