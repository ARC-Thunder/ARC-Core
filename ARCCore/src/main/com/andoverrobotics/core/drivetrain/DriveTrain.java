package com.andoverrobotics.core.drivetrain;


import com.andoverrobotics.core.utilities.IMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.util.Range;

/**
 * Defines the interface for an ordinary DriveTrain object with support for a default motor power.
 */
public abstract class DriveTrain {

  protected final OpMode opMode;
  protected double defaultPower;

  protected DriveTrain(OpMode opMode) {
    this.opMode = opMode;
  }

  /**
   * Sets the default driving power.
   *
   * @param power The default power
   */
  public final void setDefaultDrivePower(double power) {
    defaultPower = Range.clip(power, 0, 1);
  }

  /**
   * Gets the default driving power.
   *
   * @return The current value of defaultPower
   */
  public final double getDefaultDrivePower() {
    return defaultPower;
  }

  // -- Autonomous Methods --

  /**
   * Drives forwards a specific distance at the default power.
   *
   * @param distanceInInches The distance to travel, in inches
   */
  public final void driveForwards(double distanceInInches) {
    driveForwards(distanceInInches, defaultPower);
  }

  /**
   * Drives forwards a specific distance at a specific power.
   *
   * @param distanceInInches The distance to travel, in inches
   * @param power The power to drive at, between 0 and 1, inclusive
   */
  public abstract void driveForwards(double distanceInInches, double power);

  /**
   * Drives backwards a specific distance at the default power.
   *
   * @param distanceInInches The distance to travel, in inches
   */
  public final void driveBackwards(double distanceInInches) {
    driveBackwards(distanceInInches, defaultPower);
  }

  /**
   * Drives backwards a specific distance at a specific power.
   *
   * @param distanceInInches The distance to travel, in inches
   * @param power The power to drive at, between 0 and 1, inclusive
   */
  public abstract void driveBackwards(double distanceInInches, double power);

  /**
   * Rotates clockwise a specific amount, at the default power.
   *
   * @param degrees The amount to rotate (in degrees), between 0 and 360, inclusive
   */
  public final void rotateClockwise(int degrees) {
    rotateClockwise(degrees, defaultPower);
  }

  /**
   * Rotates clockwise a specific amount, at a specified power.
   *
   * @param degrees The amount to rotate (in degrees), between 0 and 360, inclusive
   * @param power The power to rotate at, between 0 and 1, inclusive
   */
  public abstract void rotateClockwise(int degrees, double power);

  /**
   * Rotates counterclockwise a specific amount, at the default power.
   *
   * @param degrees The amount to rotate (in degrees), between 0 and 360, inclusive
   */
  public final void rotateCounterClockwise(int degrees) {
    rotateCounterClockwise(degrees, defaultPower);
  }

  /**
   * Rotates counterclockwise a specific amount, at a specified power.
   *
   * @param degrees The amount to rotate (in degrees), between 0 and 360, inclusive
   * @param power The power to rotate at, between 0 and 1, inclusive
   */
  public abstract void rotateCounterClockwise(int degrees, double power);

  // -- Teleop Methods --

  /**
   * Sets the power to move at.
   *
   * @param power The power, between -1 and 1, inclusive. Positive is forwards, negative is
   * backwards
   */
  public abstract void setMovementPower(double power);

  /**
   * Sets the rotation power.
   *
   * @param power The power, between -1 and 1, inclusive. Positive is clockwise, negative is
   * counterclockwise
   */
  public abstract void setRotationPower(double power);

  /**
   * Sets the power to move at and rotate at.
   *
   * @param movePower The power to move at, between -1 and 1, inclusive. Positive is forwards,
   * negative is backwards
   * @param rotatePower The power to rotate at, between -1 and 1, inclusive. Positive is clockwise,
   * negative is counterclockwise
   */
  public abstract void setMovementAndRotation(double movePower, double rotatePower);

  protected boolean opModeIsActive() {
    boolean isAutonomous = opMode instanceof LinearOpMode;

    return !isAutonomous || ((LinearOpMode) opMode).opModeIsActive();
  }

  /**
   * Stops the motor(s).
   */
  public void stop() {
    for (IMotor motor : getMotors()) {
      motor.setPower(0);
    }
  }


  public void setMotorMode(RunMode mode) {
    for (IMotor motor : getMotors()) {
      motor.setMode(mode);
    }
  }

  // -- Internal methods

  protected abstract IMotor[] getMotors();

  protected boolean isBusy() {
    for (IMotor motor : getMotors()) {
      if (motor.isBusy()) {
        return true;
      }
    }
    return false;
  }
}
