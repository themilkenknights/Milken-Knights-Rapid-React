// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.miscellaneous;

import frc.robot.Drive;
import frc.robot.Drive.ERROR;

/** Add your docs here. */
public class TestMotors {
    private Drive mDrive = Drive.getInstance();
    private EzLogger mLog = EzLogger.getInstance();
    private MkTimerV2 mTime = new MkTimerV2(3);
    private boolean starting = true;
    private boolean drivingDone = false;
    private double driveEndTime = 0;
    private double turnEndTime = 0;
    public boolean testMotors(double speed, double angle)
    {
        if(starting)
        {
            mTime.startTimer();
            starting = false;
        }
        else if(!drivingDone && !starting)
        {
            mDrive.testDriveMotors(speed, mTime.getTime());
        }
        else if(drivingDone && !starting)
        {
            mDrive.testAngleMotors(angle, mTime.getTime());
        }
        if(mTime.isTimerDone() && !drivingDone)
        {
            driveEndTime = mTime.getTime();
            drivingDone = true;
            starting = true;
        }
        else if(mTime.isTimerDone() && drivingDone)
        {
            turnEndTime = mTime.getTime();
            mLog.writeLog("Drive End Time: " + driveEndTime);
            mLog.writeLog("Turn End Time: " + turnEndTime);
            for(int i = 0; i < 20; i++)
            {
                if(i < 10)
                {
                    mLog.writeLog("Accuracy For driveErrorTest[" + i + "]: " + Double.toString(mDrive.getError(ERROR.DRIVE, i)));
                }
                else if(i >= 10)
                {
                    mLog.writeLog("Accuracy For turnErrorTest[" + i%10 + "]: " + Double.toString(mDrive.getError(ERROR.TURN, i%10)));
                }
            }
            return false;
        }
        return true;
    }
}
