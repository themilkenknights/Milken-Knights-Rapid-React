// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.HOOD;

/**The Hood class contains everything related to the hood*/
public class Hood {
    
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

 /**Updates Hood values in shuffleboard*/
    public void updateHood()
    {
        SmartDashboard.putNumber("hood", getHoodPosition());
    }

/**Powers the hood motor at varying speeeds [-1, 1]*/
    public void setHoodPercent(double setpoint)
    {
        hood.set(setpoint);
    }
/**DIY PID */
    public void setHoodPosition(double setpoint)
    {
        hood.set(MkUtil.limitAbsolute((setpoint - hoodEncoder.getPosition()) * HOOD.hoodKP, HOOD.maxOutput));
    }

 /**Sets the hood motor's integrated encoder's position*/
    public double getHoodPosition()
    {
        return hoodEncoder.getPosition();
    }

    private static class InstanceHolder
    {
        private static final Hood mInstance = new Hood();
    } 
    
}
