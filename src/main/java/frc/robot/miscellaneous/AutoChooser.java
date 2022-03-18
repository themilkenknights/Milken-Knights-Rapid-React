// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.miscellaneous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/** Add your docs here. */
public class AutoChooser {
    private Command m_autonomousCommand;
    //private EzLogger mLog = EzLogger.getInstance();
    private enum AutoChoosers
    {
        TOPLEFT, TOP, TOPRIGHT, BOTLEFT, BOT, BOTRIGHT, NOTHING
    }
    private SendableChooser<AutoChoosers> positionChooser = new SendableChooser<>();
    private ShuffleboardTab mTab = Shuffleboard.getTab("match");
    
    public void robotInitAutoChooser(AutoChoosers... choose)
    {
        Shuffleboard.selectTab("Match");
        for(int i = 0; i < choose.length; i++)
        {
            positionChooser.addOption(choose[i].toString(), choose[i]);
        }
    }

    public void robotPeriodicAutoChooser()
    {
        CommandScheduler.getInstance().run();
        Shuffle.getInstance().update();
    }

    public void autoInitAutoChooser(AutoChoosers choose, CommandArray... arr)
    {
    //    mLog.writeLog("Autonomous Initialized");
        Shuffleboard.addEventMarker("Auto Init", EventImportance.kNormal);
        //mLog.writeLog("Running Auto: " + positionChooser.getSelected().toString());
     switch (positionChooser.getSelected()) {
         case TOPLEFT:
             m_autonomousCommand = arr[0].asSequentialCommandGroup();// new
                                                                     // DriveStr8();//m_robotContainer.getAutonomousCommand();
             break;
         case TOP:
             m_autonomousCommand = arr[1].asSequentialCommandGroup();// new
                                                                     // DriveStr8();//m_robotContainer.getAutonomousCommand();
             break;
         case TOPRIGHT:
             m_autonomousCommand = arr[2].asSequentialCommandGroup();// new
                                                                     // DriveStr8();//m_robotContainer.getAutonomousCommand();
             break;
         case BOTLEFT:
             m_autonomousCommand = arr[3].asSequentialCommandGroup();// new
                                                                     // DriveStr8();//m_robotContainer.getAutonomousCommand();
             break;
         case BOT:
             m_autonomousCommand = arr[4].asSequentialCommandGroup();// new
                                                                     // DriveStr8();//m_robotContainer.getAutonomousCommand();
             break;
         case BOTRIGHT:
             m_autonomousCommand = arr[5].asSequentialCommandGroup();// new
                                                                     // DriveStr8();//m_robotContainer.getAutonomousCommand();
             break;
       case NOTHING: 
         break;
     }
     if (m_autonomousCommand != null) {
       m_autonomousCommand.schedule();
     }
    }


    public void teleopInitAutoChooser()
    {
      //  mLog.writeLog("Teleop Initialized");
        Shuffleboard.addEventMarker("Teleop Init", EventImportance.kNormal);
        if (m_autonomousCommand != null) {
        m_autonomousCommand.cancel();
        }
    }


}
