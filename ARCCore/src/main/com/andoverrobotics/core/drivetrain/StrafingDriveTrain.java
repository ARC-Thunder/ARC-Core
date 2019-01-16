package com.andoverrobotics.core.drivetrain;

import com.andoverrobotics.core.utilities.Coordinate;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Describes the interface of a {@link DriveTrain} that is capable of "strafing," or moving the
 * robot in any direction without turning.
 */
public abstract class StrafingDriveTrain extends DriveTrain {

    protected StrafingDriveTrain(OpMode opMode) {
        super(opMode);
    }

    /**
     * Drives forwards with given power.
     *
     * @param distanceInInches The number of inches to strafe forwards
     * @param power            The power at which to strafe forwards
     */
    @Override
    public void driveForwards(double distanceInInches, double power) {
        strafeInches(0, Math.abs(distanceInInches), Math.abs(power));
    }

    /**
     * Drives backwards with given power.
     *
     * @param distanceInInches The number of inches to strafe backwards
     * @param power            The power at which to strafe backwards
     */
    @Override
    public void driveBackwards(double distanceInInches, double power) {
        strafeInches(0, -Math.abs(distanceInInches), -Math.abs(power));
    }

    /**
     * Strafes right with defaultPower.
     *
     * @param distanceInInches The number of inches to strafe right
     */
    public final void strafeRight(double distanceInInches) {
        strafeRight(distanceInInches, defaultPower);
    }

    /**
     * Strafes right at given power.
     *
     * @param distanceInInches The number of inches to strafe right
     * @param power            The power at which to strafe right
     */
    public void strafeRight(double distanceInInches, double power) {
        if (distanceInInches < 0)
            strafeLeft(-1 * distanceInInches, power);
        else
            strafeInches(distanceInInches, 0, power);
    }

    /**
     * Strafes left at default power.
     *
     * @param distanceInInches The number of inches to strafe left
     */
    public final void strafeLeft(double distanceInInches) {
        strafeLeft(distanceInInches, defaultPower);
    }

    /**
     * Strafes left at given power.
     *
     * @param distanceInInches The number of inches to strafe left
     * @param power            The power at which to strafe left
     */
    public void strafeLeft(double distanceInInches, double power) {
        if (distanceInInches < 0)
            strafeRight(-1 * distanceInInches, power);
        else
            strafeInches(-Math.abs(distanceInInches), 0, power);
    }

    /**
     * Strafes the vector <xInInches, yInInches> at the default power.
     *
     * @param xInInches The number of inches to strafe in the x-direction
     * @param yInInches The number of inches to strafe in the y-direction
     */
    public final void strafeInches(double xInInches, double yInInches) {
        strafeInches(xInInches, yInInches, defaultPower);
    }

    /**
     * Strafes the vector <xInInches, yInInches> at the given power.
     *
     * @param xInInches The number of inches to strafe in the x-direction
     * @param yInInches The number of inches to strafe in the y-direction
     * @param power     The power at which to strafe
     */
    public final void strafeInches(double xInInches, double yInInches, double power) {
        strafeInches(Coordinate.fromXY(xInInches, yInInches), power);
    }

    /**
     * Strafes the vector inchOffset at the default power.
     *
     * @param inchOffset The coordinate (relative to the current position) to strafe to
     */
    public final void strafeInches(Coordinate inchOffset) {
        strafeInches(inchOffset, defaultPower);
    }

    /**
     * Strafes the vector inchOffset at the given power.
     *
     * @param inchOffset The coordinate (relative to the current position) to strafe to
     * @param power      The power at which to strafe
     */
    public abstract void strafeInches(Coordinate inchOffset, double power);

    // -- Teleop Methods --

    /**
     * Sets the robot to strafe forward with given power.
     *
     * @param power The power, between -1 and 1, inclusive. Positive is forwards, negative is
     *              backwards
     */
    @Override
    public void setMovementPower(double power) {
        setStrafe(0, 1, power);
    }

    /**
     * Sets the robot to strafe in the direction of the vector <x, y>.
     *
     * @param x The number of inches in the x-direction to strafe.
     * @param y The number of inches in the y-direction to strafe.
     */
    public final void setStrafe(double x, double y) {
        setStrafe(Coordinate.fromXY(x, y));
    }

    /**
     * Sets the robot to strafe in the direction of (Coordinate) direction with the default power.
     *
     * @param direction The direction in which to strafe
     */
    public final void setStrafe(Coordinate direction) {
        setStrafe(direction, defaultPower);
    }

    /**
     * Sets the robot to strafe in the direction of polarDirection with the default power.
     *
     * @param polarDirection The polar direction in which to strafe
     */
    public final void setStrafe(int polarDirection) {
        setStrafe(polarDirection, defaultPower);
    }

    /**
     * Sets the robot to strafe in the direction of the vector <x, y> at the given power.
     *
     * @param x     The number of inches in the x-direction to strafe
     * @param y     The number of inches in the y-direction to strafe
     * @param power The power at which to strafe
     */
    public final void setStrafe(double x, double y, double power) {
        setStrafe(Coordinate.fromXY(x, y), power);
    }

    /**
     * Sets the robot to strafe in the direction of (Coordinate) direction at the given power.
     *
     * @param direction The direction in which to strafe
     * @param power     The power at which to strafe, between -1 and 1, inclusive
     */
    public abstract void setStrafe(Coordinate direction, double power);
}
