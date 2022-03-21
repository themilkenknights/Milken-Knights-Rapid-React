// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.miscellaneous;

import edu.wpi.first.wpilibj.Timer;

/**new mktimer*/
public class MkTimerV2 {
    private double startTime;
    private double finishTime;

    public MkTimerV2(){}

    public MkTimerV2(double finishTime)
    {
        this.finishTime = finishTime;
    }

    public void startTimer()
    {
        this.startTime = Timer.getFPGATimestamp();
    }

    public void startTimer(double finishTime)
    {
        this.finishTime = finishTime;
        this.startTime = Timer.getFPGATimestamp();
    }

    public double getTime()
    {
        return Timer.getFPGATimestamp() - this.startTime;
    }

    public double getFPGATime()
    {
        return Timer.getFPGATimestamp();
    }

    public boolean isTimerDone()
    {
        return getTime() >= this.finishTime;
    }

    public boolean isTimerDone(double finishTime)
    {
        return getTime() > finishTime;
    }

    public void setFinishTime(double finishTime)
    {
        this.finishTime = finishTime;
    }
}