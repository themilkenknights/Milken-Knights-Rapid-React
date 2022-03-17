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
    private MkTimerV2 mTimeAll = new MkTimerV2();
    private boolean starting = true;
    private boolean drivingDone = false;
    private boolean turningDone = false;
    private double driveEndTime = 0;
    private double turnEndTime = 0;

    public void reset()
    {
        starting = true;
        drivingDone = false;
        turningDone = false;
        driveEndTime = 0;
        turnEndTime = 0;
    }

    public void start()
    {
        mTimeAll.startTimer();
        reset();
    }

    public double getTime()
    {
        return mTimeAll.getTime();
    }

    public boolean testMotors(double speed, double angle)
    {
        if(starting)
        {
            mTime.startTimer();
            starting = false;
        }
        if(!drivingDone && !starting)
        {
            mDrive.testDriveMotors(speed, mTime.getTime());
        }
        else if(!turningDone && drivingDone && !starting)
        {
            mDrive.testAngleMotors(angle, mTime.getTime());
        }
        if(mTime.isTimerDone() && !drivingDone)
        {
            driveEndTime = mTime.getTime();
            drivingDone = true;
            starting = true;
        }
        else if(mTime.isTimerDone() && drivingDone && !turningDone)
        {
            starting = false;
            turningDone = true;
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
