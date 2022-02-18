// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.HOOD;

/** Add your docs here. */
public class Hood {
    /*
    private CANSparkMax hood = new CANSparkMax(HOOD.hoodCANID, MotorType.kBrushless);
    private RelativeEncoder hoodEncoder = hood.getEncoder();
    private Hood()
    {
        hood.setIdleMode(CANSparkMax.IdleMode.kBrake);
        hood.setSmartCurrentLimit(5);
    }

    public static Hood getInstance()
    {
        return InstanceHolder.mInstance;
    }

    public void updateHood()
    {
        SmartDashboard.putNumber("hood", getHoodPosition());
    }

    public void setHoodPercent(double setpoint)
    {
        hood.set(setpoint);
    }

    public void setHoodPosition(double setpoint)
    {
        hood.set(MkUtil.limitAbsolute((setpoint - hoodEncoder.getPosition()) * HOOD.hoodKP, HOOD.maxOutput));
    }

    public double getHoodPosition()
    {
        return hoodEncoder.getPosition();
    }

    private static class InstanceHolder
    {
        private static final Hood mInstance = new Hood();
    } 
    */
}
