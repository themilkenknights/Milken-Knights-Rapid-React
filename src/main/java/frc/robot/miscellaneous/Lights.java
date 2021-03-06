// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.miscellaneous;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Drive;
import frc.robot.Constants.LIGHTS;

/**The Lights class contains everything relating to ergabled*/
public class Lights {
//navx vlaue thing and light offset values so i dont have to create a new variable every time the function runs
    private double navXRot = 0;
    private int offset;

    private AddressableLED LEDS = new AddressableLED(LIGHTS.PWMPORT);
    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(LIGHTS.bufferNum);
    private Drive mDrive = Drive.getInstance();
    private Timer timer = new Timer();

    private Lights()
    {
        LEDS.setLength(LIGHTS.bufferNum);
        LEDS.setData(buffer);
        LEDS.start();
    }

    public static Lights getInstance()
    {
        return InstanceHolder.mInstance;
    }
    
 /**oui oui*/
    public void french() {
        // For every pixel
        for (var i = 0; i < LIGHTS.bufferNum; i++) 
        {   
                timer.start();
                if(i < (LIGHTS.bufferNum / 3))
                {
                    buffer.setRGB((i+offset)%LIGHTS.bufferNum, 0, 0, LIGHTS.MaxRGBValue);
                }
                else if(i >= (LIGHTS.bufferNum / 3) && i < ((2 * LIGHTS.bufferNum) / 3))
                {
                    buffer.setRGB((i+offset)%LIGHTS.bufferNum, LIGHTS.MaxRGBValue, LIGHTS.MaxRGBValue, LIGHTS.MaxRGBValue);
                }
                else
                {
                    buffer.setRGB((i+offset)% LIGHTS.bufferNum, LIGHTS.MaxRGBValue, 0, 0);
                }
                if(timer.get() > 0.08)
                {
                    offset = (offset + 1) % LIGHTS.bufferNum;
                    timer.reset();
                    LEDS.setData(buffer);
                }
        }
      }

   /**me version*/
      public void lilNavX()
      {
          navXRot = (((mDrive.getNavx() + 360) % 360) * LIGHTS.bufferNum/360);
          for(double i = 0; i < LIGHTS.bufferNum; i++)
          {
              buffer.setRGB(((int)i + 100) % 100, 0, 0, 0);
              if(!(i > (((navXRot + LIGHTS.bufferNum) % LIGHTS.bufferNum) + 5) || i < (((navXRot + LIGHTS.bufferNum) % LIGHTS.bufferNum) - 5)))
              {
                buffer.setRGB(((int)i + 100) % 100, LIGHTS.MaxRGBValue, LIGHTS.MaxRGBValue, LIGHTS.MaxRGBValue);
              }
              if(navXRot < 5 || navXRot > 95)
              {
                for(double j = navXRot - 5; j < navXRot + 5; j++)
                {
                    buffer.setRGB(((int)j + 100) % 100, LIGHTS.MaxRGBValue, LIGHTS.MaxRGBValue, LIGHTS.MaxRGBValue);
                }
              } 
                
          }
          LEDS.setData(buffer);
      }

   /**setphan version*/
      public void lilNavXTWO()
      {
          //TODO add 90 somewhere here to make forward true forward
        navXRot = (((mDrive.getNavx() + 360) % 360) * LIGHTS.bufferNum/360);
        for(int i = 0; i < LIGHTS.bufferNum; i++)
        {
            if(((i + 10 / 2 - (int)navXRot) + LIGHTS.bufferNum) % LIGHTS.bufferNum < 10)
            {
                buffer.setRGB(i, 255/2, 255/2, 255/2);
            }
            else
            {
                buffer.setRGB(i, 0, 0, 0);
            }
        }
        LEDS.setData(buffer);
      }

/**
 * displays voltage on the leds
 * @param volts volt value from driver station
 */
      public void voltage(double volts)
      {
          for(int i = 0; i < LIGHTS.bufferNum; i++)
          {
              buffer.setRGB(i, (int)(((((volts - 11)/13) * LIGHTS.MaxRGBValue))/2), 0, 0);
          }
          LEDS.setData(buffer);
      }




      public void ukraine()
      {
        for (var i = 0; i < LIGHTS.bufferNum; i++) 
        {   
                timer.start();
                if(i < (LIGHTS.bufferNum / 4))
                {
                    buffer.setRGB((i+offset)%LIGHTS.bufferNum, 0, 0, LIGHTS.MaxRGBValue);
                }
                else if(i >= (LIGHTS.bufferNum / 4) && i < ((2 * LIGHTS.bufferNum) / 4))
                {
                    buffer.setRGB((i+offset)%LIGHTS.bufferNum, LIGHTS.MaxRGBValue, LIGHTS.MaxRGBValue, 0);
                }
                else if(i >= (LIGHTS.bufferNum / 3) && i < ((2 * LIGHTS.bufferNum) / 3))
                {
                    buffer.setRGB((i+offset)%LIGHTS.bufferNum, 0, 0, LIGHTS.MaxRGBValue);
                }
                else
                {
                    buffer.setRGB((i+offset)% LIGHTS.bufferNum, LIGHTS.MaxRGBValue, LIGHTS.MaxRGBValue, 0);
                }
                if(timer.get() > 0.08)
                {
                    offset = (offset + 1) % LIGHTS.bufferNum;
                    timer.reset();
                    LEDS.setData(buffer);
                }
        }
      }

      public void off()
      {
        for(int i = 0; i < LIGHTS.bufferNum; i++)
        {
            buffer.setRGB(i, 0, 0, 0);
        }
        LEDS.setData(buffer);
      }


      public void teamMode(Alliance all)
      {
          if(all == Alliance.Blue)
          {
            for(int i = 0; i < LIGHTS.bufferNum; i++)
            {
                buffer.setRGB(i, 0, 0, LIGHTS.MaxRGBValue);
            }
            LEDS.setData(buffer);
          }
          else
          {
            for(int i = 0; i < LIGHTS.bufferNum; i++)
            {
                buffer.setRGB(i, LIGHTS.MaxRGBValue, 0, 0);
            }
            LEDS.setData(buffer);
          }
      }

    private static class InstanceHolder
    {
        private static final Lights mInstance = new Lights();
    } 
}
