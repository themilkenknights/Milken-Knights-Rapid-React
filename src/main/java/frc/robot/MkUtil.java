// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DRIVE;
import frc.robot.Constants.TURN;

public class MkUtil {
//original ones 
/*
 public static double nativeToInches(double nativeUnits) {
    return (nativeUnits / 2048.0) * DRIVE.kWheelCircumference;
  }

  public static double inchesToNative(double in) {
    return (in / DRIVE.kWheelCircumference) * 2048.0;
  }*/

  //new ones with greer ration 

  /**
   * (Falcon) native units to inches
   * <p>
   * specifically made for the driving motors
   * @param nativeUnits units of native
   * @return inches
   */
  public static double nativeToInches(double nativeUnits) {
    return (nativeUnits / (2048.0 * DRIVE.greerRatio)) * DRIVE.kWheelCircumference;
  }
  /**
   * inches to native units (Falcon)
   * <p>
   * specifically made for the driving motors
   * @param in inches
   * @return native units
   */
  public static double inchesToNative(double in) {
    return (in / DRIVE.kWheelCircumference) * (2048.0 * DRIVE.greerRatio);
  }
  /**
   * native units (1n/100s) to inches (1i/1s)
   * @param vel motor velocity (native units)
   * @return velocity of motors in inches
   */
  public static double nativePer100MstoInchesPerSec(double vel) {
    return 10 * nativeToInches(vel);
  }
  /**
   * inches (1i/1s) to native (1n/100s)
   * @param vel motor velocity (inches)
   * @return velocity of motors in native units
   */
  public static double inchesPerSecToUnitsPer100Ms(double vel) {
    return inchesToNative(vel) / 10;
  }
  /**
   * inches to meters
   * @param inches inches
   * @return meters
   */
  public static double inchesToMeters(double inches) {
    return Units.inchesToMeters(inches);
  }
  /** native units to meters
   * @param nativeUnits units of native
   * @return meters
   */
  public static double nativeToMeters(double nativeUnits) {
    return inchesToMeters(nativeToInches(nativeUnits));
  }
  /**
   * native units (1n/100s) to meters (1m/1s)
   * @param nativeUnits units of native
   * @return velocity of motors in meters
   */
  public static double nativePer100MsToMetersPerSec(double nativeUnits) {
    return inchesToMeters(nativePer100MstoInchesPerSec(nativeUnits));
  }
  /**
   * meters to inches
   * @param meters meters
   * @return inches
   */
  public static double metersToInches(double meters) {
    return Units.metersToInches(meters);
  }
  /**
   * meters (1m/1s) to native units (1n/100s)
   * @param meters meters
   * @return velocity of motors in native units
   */
  public static double metersPerSecondToNativeUnitsPer100Ms(double meters) {
    return inchesPerSecToUnitsPer100Ms(metersToInches(meters));
  }
  /**
   * cheesy drive
   * @param throttle
   * @param wheel
   * @param cubeInputs
   * @return cheesy drive?
   */
  public static DriveSignal cheesyDrive(double throttle, double wheel, boolean cubeInputs) {
    double kThrottleDeadband = 0.0;
    double kWheelDeadband = 0.003;
    double leftMotorSpeed;
    double rightMotorSpeed;
    double moveValue = limitAbsolute(throttle, 1.0);
    double rotateValue = limitAbsolute(wheel, 1.0);
    moveValue = deadband(moveValue, kThrottleDeadband);
    rotateValue = deadband(rotateValue, kWheelDeadband);
    if (cubeInputs) {
      rotateValue = rotateValue * rotateValue * rotateValue;
    }
    rotateValue = rotateValue / 2.3;
    if (moveValue > 0.0) {
      if (rotateValue > 0.0) {
        leftMotorSpeed = moveValue - rotateValue;
        rightMotorSpeed = Math.max(moveValue, rotateValue);
      } else {
        leftMotorSpeed = Math.max(moveValue, -rotateValue);
        rightMotorSpeed = moveValue + rotateValue;
      }
    } else {
      if (rotateValue > 0.0) {
        leftMotorSpeed = -Math.max(-moveValue, rotateValue);
        rightMotorSpeed = moveValue + rotateValue;
      } else {
        leftMotorSpeed = moveValue - rotateValue;
        rightMotorSpeed = -Math.max(-moveValue, -rotateValue);
      }
    }
    return new DriveSignal(leftMotorSpeed, rightMotorSpeed);
  }
  /**
   * limits a value between min and max
   * @param value your value
   * @param min min
   * @param max max....
   * @return your value, capped between min and max
   */
  public static double limit(double value, double min, double max) {
    if (value > max) {
      return max;
    } else if (value < min) {
      return min;
    } else {
      return value;
    }
  }
  /**
   * caps a value so its above a deadband or zero if below said deadband
   * @param val your value
   * @param deadband desired deadband
   * @return value or zero depending on deadband
   */
  private static double deadband(double val, double deadband) {
    return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
  }
  /**
   * caps a value so its below a max value or it is said max value, with the same sign as the original value
   * @param a input value
   * @param max max
   * @return value or max with the sign of value
   */
  public static double limitAbsolute(double a, double max) {
    return Math.abs(a) < max ? a : Math.copySign(max, a);
  }
  /**
   * cheesy drive
   */
  public static class DriveSignal {

    public static DriveSignal STOP = new DriveSignal(0, 0);
    protected double mLeftMotor;
    protected double mRightMotor;

    public DriveSignal(double left, double right) {
      mLeftMotor = left;
      mRightMotor = right;
    }

    public double getLeft() {
      return mLeftMotor;
    }

    public double getRight() {
      return mRightMotor;
    }

    @Override
    public String toString() {
      return "L: " + mLeftMotor + " R: " + mRightMotor;
    }
  }

  //!     this is ben binders mkutil extension, above is swerdlow code

  /**
   * (Falcon) native units to degrees
   * @param gimmeRots rotations of Falcon
   * @param greerRatio gear ratio 
   * @return degrees
   * @author me
   */
  public static double nativeToDegrees(double gimmeRots, double greerRatio)
  {
    return (gimmeRots * 360) / (greerRatio * Constants.oneEncoderRotation);
  }
  /**
   * degrees to native (Falcon)
   * @param gimmeDeg degrees
   * @param greerRatio gear ratio
   * @return native units
   */
  public static double degreesToNative(double gimmeDeg, double greerRatio)
  {
    return (gimmeDeg * Constants.oneEncoderRotation * greerRatio) / 360;
  }

  /**
   * double to boolean, since java wont let me
   * @param bool value
   * @return true if one, false if zero
   */
  public static boolean doubleToBoolean(double bool)
  {
    if(bool == 1.0)
    {
      return true;
    }
    else if(bool == 0.0)
    {
      return false;
    }
    else
    {
      return false;
    }
  }
  /**
   * takes in two numbers, and returns the second number with the first one's sign
   * @param number number whose sign will be checked
   * @param numberTwo number that will be returned
   * @return
   */
  public static double isPositive(double number, double numberTwo)
  {
    if(number < 0.0)
    {
      return -1.0 * numberTwo;
    }
    else if(number > 0.0)
    {
      return 1.0 * numberTwo;
    }
    else
    {
      return numberTwo;
    }
  }

  //!     stolen from team 6624
  
    /**
     * "Get the closest angle between the given angles."
     * @param a angle a
     * @param b angle b
     * @return angle closest between the two angles
     * @author team 6624
     */
  public static double closestAngle(double a, double b)
  {
          double dir = (b % 360.0) - (a % 360.0);

          // convert from -360 to 360 to -180 to 180
          if (Math.abs(dir) > 180.0)
          {
                  dir = -(Math.signum(dir) * 360.0) + dir;
          }
          return dir;
  }
  

  //!     also stolen from 6624
  /**
   * gives the closest path to a given setpoint based on the Falcon's position
   * @param talon Falcon that will be moved
   * @param setpoint angle setpoint
   * @param greerRatio gear ratio
   * @return returns a new setpoint for the Falcon to turn to
   * @author team 6624
   */
  public static double setDirection(TalonFX talon, double setpoint, double greerRatio)
  {
    // use the fastest way
    double currentAngle = MkUtil.nativeToDegrees((talon.getSelectedSensorPosition()), greerRatio);
    return currentAngle + closestAngle(currentAngle, setpoint);
  }  
  /**
   * same as setDirection(TalonFX) except this uses a CANCoder
   * @param coder CANCoder
   * @param setpoint angle setpoint
   * @return returns a new setpoint for the Falcon to turn to
   * @author team 6624
   */
  public static double setDirection(CANCoder coder, double setpoint)
  {
    // use the fastest way
    double currentAngle = coder.getAbsolutePosition();
    return currentAngle + closestAngle(currentAngle, setpoint);
  }



  /**
   * decides whether a driving motor should flip based on where the angular motor's setpoint is, and also returns a new setpoint based on the motors position
   * @param talon angular motor
   * @param setpoint setpoint for angular motor
   * @param pid bogus driving pid thats only used for this, not for actual calculations
   * @return returns best angle of travel for the angular motor, as well as changing the flip value of the driving motor
   * @author team 6624
   */
  public static double setDirection(TalonFX talon, double setpoint, PIDController pid)
    {
        double currentAngle = nativeToDegrees(talon.getSelectedSensorPosition(), TURN.greerRatio);
        // find closest angle to setpoint
        double setpointAngle = closestAngle(currentAngle, setpoint);
        // find closest angle to setpoint + 180
        double setpointAngleFlipped = closestAngle(currentAngle, setpoint + 180.0);
        // if the closest angle to setpoint is shorter
        if (Math.abs(setpointAngle) <= Math.abs(setpointAngleFlipped))
        {
            // unflip the motor direction use the setpoint
            pid.setP(Math.abs(pid.getP()) * 1.0);
            return (currentAngle + setpointAngle);
        }
        // if the closest angle to setpoint + 180 is shorter
        else
        {
            // flip the motor direction and use the setpoint + 180
            pid.setP(Math.abs(pid.getP()) * -1.0);
            return (currentAngle + setpointAngleFlipped);
        }
    }

    /**
     * same as setDirection(TalonFX) except this uses a CANCoder
     * @param coder CANCoder
     * @param setpoint setpoint for angular motor
     * @param pid bogus driving pid thats only used for this, not for actual calculations
     * @return returns best angle of travel for the angular motor, as well as changing the flip value of the driving motor
     * @author team 6624
     */
    public static double setDirection(CANCoder coder, double setpoint, PIDController pid)
    {
        double currentAngle = coder.getAbsolutePosition();
        // find closest angle to setpoint
        double setpointAngle = closestAngle(currentAngle, setpoint);
        // find closest angle to setpoint + 180
        double setpointAngleFlipped = closestAngle(currentAngle, setpoint + 180.0);
        // if the closest angle to setpoint is shorter
        if (Math.abs(setpointAngle) <= Math.abs(setpointAngleFlipped))
        {
            // unflip the motor direction use the setpoint
            pid.setP(Math.abs(pid.getP()) * 1.0);
            return (currentAngle + setpointAngle);
        }
        // if the closest angle to setpoint + 180 is shorter
        else
        {
            // flip the motor direction and use the setpoint + 180
            pid.setP(Math.abs(pid.getP()) * -1.0);
            return (currentAngle + setpointAngleFlipped);
        }
    }

    //TODO hope these work
    /**
     * converts native velocity (1n/100ms) into degree velocity (1d/1s)
     * @param gimmeRots velocity (1n/100ms)
     * @param greerRatio gear ratio
     * @return velocity (degrees per second)
     * @apiNote i dont know if these are correct
     */
    public static double nativePer100MstoDegreesPerSec(double gimmeRots, double greerRatio)
    {
      return (gimmeRots * 1000 * 360) / (greerRatio * 2048);
    }

    /**
     * converts degree velocity (1d/1s) into native velocity (1n/100ms)
     * @param gimmeDeg velocity (1d/1s)
     * @param greerRatio gear ratio
     * @return velocity (native per 100ms)
     * @apiNote i dont know if these are correct
     */
    public static double degreesPerSectoNativePer100Ms(double gimmeDeg, double greerRatio)
    {
      return (gimmeDeg * 2048 * greerRatio) / (360 * 100);
    }

}
