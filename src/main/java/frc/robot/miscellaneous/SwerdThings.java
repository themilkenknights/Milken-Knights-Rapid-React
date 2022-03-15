// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.miscellaneous;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.hal.simulation.DriverStationDataJNI;
import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**Update values for robot saftey things?*/
public class SwerdThings {

    private TalonFX testerFalcon = new TalonFX(999);

    public void RoboRioDataJNIUpdate()
    {
        SmartDashboard.putNumber("RoboRioGetBrownoutVoltage", RoboRioDataJNI.getBrownoutVoltage());
        SmartDashboard.putBoolean("RoboRioGetFPGAButton",RoboRioDataJNI.getFPGAButton());
        SmartDashboard.putNumber("RoboRioGetVInCurrent",RoboRioDataJNI.getVInCurrent());
        SmartDashboard.putNumber("RoboRioGetVInVoltage",RoboRioDataJNI.getVInVoltage());
    }


    public void RobotControllerUpdate()
    {
        SmartDashboard.putNumber("RobotControllerGetBatteryVoltage", RobotController.getBatteryVoltage());
        SmartDashboard.putNumber("RobotControllerGetBrownoutVoltage", RobotController.getBrownoutVoltage());
        //SmartDashboard.putData("RobotControllerGetCanStatus",RobotController.getCANStatus().); <-- driver station data
        SmartDashboard.putNumber("RobotControllerGetFPGARevision",RobotController.getFPGARevision());
        SmartDashboard.putNumber("RobotControllerGetFPGATime",RobotController.getFPGATime());
        SmartDashboard.putNumber("RobotControllerGetFPGAVersion",RobotController.getFPGAVersion());
        SmartDashboard.putBoolean("RobotControllerGetUserButton",RobotController.getUserButton());
        SmartDashboard.putBoolean("RobotControllerIsSysActive",RobotController.isSysActive());
        SmartDashboard.putBoolean("RobotControllerIsBrownedOut",RobotController.isBrownedOut());
    } 


    public void RoboRioSimUpdate()
    {
        SmartDashboard.putNumber("RoboRioSimGetBrownoutVoltage",RoboRioSim.getBrownoutVoltage());
        SmartDashboard.putBoolean("RoboRioSimGetFPGAButton",RoboRioSim.getFPGAButton());
        SmartDashboard.putNumber("RoboRioSimGetVInCurrent",RoboRioSim.getVInCurrent());
        SmartDashboard.putNumber("RoboRioSimGetVInVoltage",RoboRioSim.getVInVoltage());
    }




    public void RobotStateUpdate()
    {
        SmartDashboard.putBoolean("RobotStateIsEStopped",RobotState.isEStopped());
        SmartDashboard.putBoolean("RobotStateIsEnabled",RobotState.isEnabled());
    }

    
    public void TalonFXUpdateSafety(TalonFX test)
    {
        SmartDashboard.putNumber("TalonFXGetBaseID",test.getBaseID());
        SmartDashboard.putNumber("TalonFXGetBusVoltage",test.getBusVoltage());
        SmartDashboard.putNumber("TalonFXGetFirmwareVersion",test.getFirmwareVersion());
        SmartDashboard.putNumber("TalonFXGetHandle",test.getHandle());
        SmartDashboard.putNumber("TalonFXGetStatorCurrent",test.getStatorCurrent());
        SmartDashboard.putNumber("TalonFXGetSupplyCurrent",test.getSupplyCurrent());
        SmartDashboard.putNumber("TalonFXGetTemperature",test.getTemperature());
        SmartDashboard.putNumber("TalonFXGetHashCode",test.hashCode());
    }
}
