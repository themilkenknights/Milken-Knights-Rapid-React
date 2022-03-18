// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.miscellaneous;

import frc.robot.Drive;
import frc.robot.Shooter;
import frc.robot.Drive.ERROR;

/** Add your docs here. */
public class TestMotors {
    private Drive mDrive = Drive.getInstance();
    private Shooter mShoot = Shooter.getInstance();
    private EzLogger mLog = EzLogger.getInstance();
    private MkTimerV2 mTime = new MkTimerV2(3);
    private MkTimerV2 mTimeAll = new MkTimerV2();
    private boolean starting = true;
    private boolean drivingDone = false;
    private boolean turningDone = false;
    private boolean mechanismDone = false;

    public void resetAll()
    {
        starting = true;
        drivingDone = false;
        turningDone = false;
        mechanismDone = false;
    }

    public void start()
    {
        mTimeAll.startTimer();
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
        else if(!drivingDone && !starting)
        {
            mDrive.testDriveMotors(speed, mTime.getTime());
        }
        else if(!turningDone && drivingDone && !starting)
        {
            mDrive.testAngleMotors(angle, mTime.getTime());
        }
        if(mTime.isTimerDone() && !drivingDone)
        {
            drivingDone = true;
            starting = true;
        }
        else if(mTime.isTimerDone() && drivingDone && !turningDone)
        {
            starting = false;
            turningDone = true;
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
            mDrive.resetDrive();
            mDrive.encoderZero();
            return true;
        }
        return false;
    }




    public boolean testMechanism(double setpoint, MECHANISM mechanism)
    {
        if(starting)
        {
            mTime.startTimer();
            starting = false;
        }
        else if(!mechanismDone && !starting)
        {
            switch(mechanism)
            {
                case Shooter:
                    mShoot.shootErrorTestVelocity(setpoint, mTime.getTime());
                    break;
                default:
                    mLog.writeLog("tf is this enum in test mechanism");
                    return false;
            }
        }
        if(mTime.isTimerDone() && !mechanismDone)
        {
            starting = false;
            mechanismDone = true;
            switch(mechanism)
            {
                case Shooter:
                    for(int i = 0; i < 6; i++)
                    {
                        mLog.writeLog("Accuracy For shootErrorTest[" + i + "]: " + Double.toString(mShoot.getError(i)));
                    }
                    break;
                default: 
                    mLog.writeLog("tf is this enum in test mechanism");
                    return false;
            }
            return true;
        }
        return false;
    }



    public enum MECHANISM 
    {
        Shooter, Climber, Intake, Elevator
    }
}
