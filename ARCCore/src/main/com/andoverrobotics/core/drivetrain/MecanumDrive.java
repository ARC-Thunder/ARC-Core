package com.andoverrobotics.core.drivetrain;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;

import com.andoverrobotics.core.utilities.Converter;
import com.andoverrobotics.core.utilities.Coordinate;
import com.andoverrobotics.core.utilities.IMotor;
import com.andoverrobotics.core.utilities.MotorPair;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Implements the {@link StrafingDriveTrain} for a Mecanum drivetrain. <p> See {@link
 * #fromOctagonalMotors(DcMotor, DcMotor, DcMotor, DcMotor, OpMode, int, int)} and {@link
 * #fromCrossedMotors(DcMotor, DcMotor, DcMotor, DcMotor, OpMode, int, int)} for instructions about
 * easier construction.
 */
public class MecanumDrive extends StrafingDriveTrain {

    private final IMotor leftDiagonal, rightDiagonal, leftSide, rightSide;

    private final int ticksPerInch;
    private final int ticksPer360;

    /**
     * Constructs a new <code>MecanumDrive</code> instance with the given {@link IMotor}s and encoder
     * parameters.
     *
     * @param leftDiagonal  The {@link IMotor} that represents the two physical motors that make the
     *                      robot go (forward and) to the left when the power is positive
     * @param rightDiagonal The {@link IMotor} that represents the two physical motors that make the
     *                      robot go (forward and) to the right when the power is positive
     * @param leftSide      The {@link IMotor} that represents the two physical motors that are on the left
     *                      side of the robot
     * @param rightSide     The {@link IMotor} that represents the two physical motors that are on the
     *                      right side of the robot
     * @param opMode        The main {@link OpMode}
     * @param ticksPerInch  The number of encoder ticks required to cause a diagonal displacement of 1
     *                      inch for the robot
     * @param ticksPer360   The number of encoder ticks required to cause a full rotation for the robot,
     *                      when this amount is applied to the left and right sides in opposite directions
     */
    public MecanumDrive(IMotor leftDiagonal, IMotor rightDiagonal, IMotor leftSide, IMotor rightSide,
                        OpMode opMode, int ticksPerInch, int ticksPer360) {

        super(opMode);

        this.leftDiagonal = leftDiagonal;
        this.rightDiagonal = rightDiagonal;
        this.leftSide = leftSide;
        this.rightSide = rightSide;

        this.ticksPerInch = ticksPerInch;
        this.ticksPer360 = ticksPer360;
    }

    /**
     * Constructs a new MecanumDrive instance that uses the given physical motors, which are arranged
     * in an octagonal manner. <p> <h2>The Octagonal Configuration</h2> In the octagonal configuration,
     * the front left and the back right motors cause the robot to move forward and to the right
     * (diagonally), and the other two motors cause the robot to move forward and to the left.
     * <p>
     * It is named to be "octagonal" because of its diagram:
     * <pre>
     *   /-\
     *   | |
     *   \-/
     * </pre>
     *
     * @param motorFL      The motor located at the front-left of the robot
     * @param motorFR      The motor located at the front-right of the robot
     * @param motorBL      The motor located at the rear-left of the robot
     * @param motorBR      The motor located at the rear-right of the robot
     * @param opMode       The main {@link OpMode}
     * @param ticksPerInch The number of encoder ticks required to cause a diagonal displacement of 1
     *                     inch for the robot
     * @param ticksPer360  The number of encoder ticks required to cause a full rotation for the robot,
     *                     when this amount is applied to the left and right sides in opposite directions
     * @return The new MecanumDrive instance
     */
    public static MecanumDrive fromOctagonalMotors(DcMotor motorFL, DcMotor motorFR, DcMotor motorBL,
                                                   DcMotor motorBR,
                                                   OpMode opMode, int ticksPerInch, int ticksPer360) {
        // /-\
        // | |
        // \-/

        return new MecanumDrive(
                MotorPair.of(motorFR, motorBL),
                MotorPair.of(motorFL, motorBR),
                MotorPair.of(motorFL, motorBL),
                MotorPair.of(motorFR, motorBR), opMode, ticksPerInch, ticksPer360);
    }

    /**
     * Constructs a new MecanumDrive instance that uses the given physical motors, which are arranged
     * in a a crossed manner. <p> <b>The Crossed Configuration</b> In the crossed configuration, the
     * front left and the back right motors cause the robot to move forward and to the left
     * (diagonally), and the other two motors cause the robot to move forward and to the right.
     * <p>
     * It is named to be "octagonal" because of its diagram:
     * <pre>
     *   \-/
     *   | |
     *   /-\
     * </pre>
     *
     * @param motorFL      The motor located at the front-left of the robot
     * @param motorFR      The motor located at the front-right of the robot
     * @param motorBL      The motor located at the rear-left of the robot
     * @param motorBR      The motor located at the rear-right of the robot
     * @param opMode       The main {@link OpMode}
     * @param ticksPerInch The number of encoder ticks required to cause a diagonal displacement of 1
     *                     inch for the robot
     * @param ticksPer360  The number of encoder ticks required to cause a full rotation for the robot,
     *                     when this amount is applied to the left and right sides in opposite directions
     * @return The new MecanumDrive instance
     */
    public static MecanumDrive fromCrossedMotors(DcMotor motorFL, DcMotor motorFR, DcMotor motorBL,
                                                 DcMotor motorBR,
                                                 OpMode opMode, int ticksPerInch, int ticksPer360) {
        // \-/
        // | |
        // /-\

        return new MecanumDrive(
                MotorPair.of(motorFL, motorBR),
                MotorPair.of(motorFR, motorBL),
                MotorPair.of(motorFL, motorBL),
                MotorPair.of(motorFR, motorBR), opMode, ticksPerInch, ticksPer360);
    }

    // Rotates the given displacement by 45deg clockwise, assigns its components to the diagonals as
    // a tick offset (setTargetPosition), then scales its components down such that the greatest
    // component is equal to the power given, followed by assigning these components to the diagonals
    // as power.
    private void driveWithEncoder(Coordinate displacement, double power) {
        double clippedPower = Range.clip(power, -1, 1);

        if (displacement.getPolarDistance() < 1e-5) {
            return;
        }

        Coordinate diagonalOffsets = displacement.rotate(-45);
        double maxOffset = Math.max(diagonalOffsets.getX(), diagonalOffsets.getY());

        int leftOffset = (int) (diagonalOffsets.getY() * ticksPerInch),
                rightOffset = (int) (diagonalOffsets.getX() * ticksPerInch);

        double leftPower = clippedPower * (diagonalOffsets.getY() / maxOffset),
                rightPower = clippedPower * (diagonalOffsets.getX() / maxOffset);

        leftDiagonal.startRunToPosition(leftOffset, Math.abs(leftPower));
        rightDiagonal.startRunToPosition(rightOffset, Math.abs(rightPower));

        while (isBusy() && opModeIsActive()) {
        }

        stop();
        setMotorMode(RUN_USING_ENCODER);
    }

    @Override
    public void rotateClockwise(int degrees, double power) {
        if (degrees < 0)
            rotateCounterClockwise(-degrees, power);
        else
            rotateWithEncoder(-Converter.normalizedDegrees(degrees), -Math.abs(power));
    }

    @Override
    public void rotateCounterClockwise(int degrees, double power) {
        if (degrees < 0)
            rotateClockwise(-degrees, power);
        else
            rotateWithEncoder(Converter.normalizedDegrees(degrees), Math.abs(power));
    }

    // Positive input means counter-clockwise
    private void rotateWithEncoder(double degrees, double power) {
        double clippedPower = Math.abs(Range.clip(power, -1, 1));
        double rotationTicks = degrees / 360.0 * ticksPer360;

        leftSide.startRunToPosition((int) -rotationTicks, clippedPower);
        rightSide.startRunToPosition((int) rotationTicks, clippedPower);

        while (isBusy() && opModeIsActive()) {
        }

        stop();
        setMotorMode(RUN_USING_ENCODER);
    }

    @Override
    public void strafeInches(Coordinate inchOffset, double power) {
        driveWithEncoder(inchOffset, power);
    }

    // -- TeleOp methods --

    @Override
    public void setMovementPower(double power) {
        double clippedPower = Range.clip(power, -1, 1);

        setMotorMode(RUN_WITHOUT_ENCODER);

        leftSide.setPower(clippedPower);
        rightSide.setPower(clippedPower);
    }

    @Override
    public void setRotationPower(double power) { //clockwise if power is positive
        double clippedPower = Range.clip(power, -1, 1);

        setMotorMode(RUN_WITHOUT_ENCODER);

        leftSide.setPower(clippedPower);
        rightSide.setPower(-clippedPower);
    }

    @Override
    public void setStrafe(Coordinate offset, double unscaledPower) {
        double direction = Converter.degreesToRadians(offset.getPolarDirection() - 45);
        double magnitude = Math.min(1, offset.getPolarDistance());
        double power = Range.clip(unscaledPower, -1, 1);

        setMotorMode(RUN_WITHOUT_ENCODER);

        leftDiagonal.setPower(Math.sin(direction) * magnitude * Math.abs(power));
        rightDiagonal.setPower(Math.cos(direction) * magnitude * Math.abs(power));
    }

    @Override
    public void setMovementAndRotation(double movePower, double rotatePower) {
        setMotorMode(RUN_WITHOUT_ENCODER);

        double leftPower = movePower + rotatePower,
                rightPower = movePower - rotatePower;

        double maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (maxPower > 1) {
            leftPower /= maxPower;
            rightPower /= maxPower;
        }

        leftSide.setPower(leftPower);
        rightSide.setPower(rightPower);
    }

    @Override
    protected IMotor[] getMotors() {
        return new IMotor[]{leftDiagonal, rightDiagonal, leftSide, rightSide};
    }
}
