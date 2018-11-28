package com.andoverrobotics.core.drivetrain;

import com.andoverrobotics.core.utilities.Converter;
import com.andoverrobotics.core.utilities.IMotor;
import com.andoverrobotics.core.utilities.MotorAdapter;
import com.andoverrobotics.core.utilities.MotorPair;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;

/**
 * Implements the tank drive DriveTrain for either two motors or four motors. <p> See {@link
 * #fromMotors(DcMotor, DcMotor, OpMode, int, int)} and {@link #fromMotors(DcMotor, DcMotor,
 * DcMotor, DcMotor, OpMode, int, int)} for instructions about easier construction.
 */
public class TankDrive extends DriveTrain {

  private final int ticksPerInch;
  private final int ticksPer360;
  private IMotor motorL;
  private IMotor motorR;

  // Verify that the motor(s) on one side is reversed if the motors point in opposite directions!

  /**
   * Creates a TankDrive from two IMotors
   *
   * @param motorL       The left IMotor
   * @param motorR       The right IMotor
   * @param opMode       The OpMode to set
   * @param ticksPerInch The number of encoder ticks required to cause a diagonal displacement of 1
   *                     inch for the robot
   * @param ticksPer360  The number of encoder ticks required to cause a full rotation for the robot,
   *                     when this amount is applied to the left and right sides in opposite directions
   */
  public TankDrive(IMotor motorL, IMotor motorR, OpMode opMode,
                   int ticksPerInch, int ticksPer360) {
    super(opMode);

    this.motorL = motorL;
    this.motorR = motorR;
    this.ticksPerInch = ticksPerInch;
    this.ticksPer360 = ticksPer360;
  }

  /**
   * Creates a TankDrive from two DcMotors.
   *
   * @param motorL       The left DcMotor
   * @param motorR       The right DcMotor
   * @param opMode       The OpMode to set
   * @param ticksPerInch The number of encoder ticks required to cause a diagonal displacement of 1
   *                     inch for the robot
   * @param ticksPer360  The number of encoder ticks required to cause a full rotation for the robot,
   *                     when this amount is applied to the left and right sides in opposite directions
   * @return A TankDrive created with the inputted motors
   */
  public static TankDrive fromMotors(DcMotor motorL, DcMotor motorR, OpMode opMode,
                                     int ticksPerInch, int ticksPer360) {

    return new TankDrive(new MotorAdapter(motorL),
            new MotorAdapter(motorR), opMode, ticksPerInch, ticksPer360);
  }

  /**
   * Creates a TankDrive from four DcMotors.
   *
   * @param motorL1      One of the left DcMotors
   * @param motorL2      The other left DcMotor
   * @param motorR1      One of the right DcMotors
   * @param motorR2      The other right DcMotor
   * @param opMode       The OpMode to set
   * @param ticksPerInch The number of encoder ticks required to cause a diagonal displacement of 1
   *                     inch for the robot
   * @param ticksPer360  The number of encoder ticks required to cause a full rotation for the robot,
   *                     when this amount is applied to the left and right sides in opposite directions
   * @return A TankDrive created using MotorPairs
   */
  public static TankDrive fromMotors(DcMotor motorL1, DcMotor motorL2, DcMotor motorR1,
                                     DcMotor motorR2,
                                     OpMode opMode, int ticksPerInch, int ticksPer360) {

    return new TankDrive(
            MotorPair.of(motorL1, motorL2),
            MotorPair.of(motorR1, motorR2), opMode, ticksPerInch, ticksPer360);
  }

  @Override
  public void driveForwards(double distanceInInches, double power) {
    driveWithEncoder(Math.abs(distanceInInches), Math.abs(power));
  }

  private void driveWithEncoder(double displacementInInches, double givenPower) {

    if (givenPower == 0) {
      stop();
      return;
    }

    double power = Math.abs(Range.clip(givenPower, -1, 1));

    if (displacementInInches < 0) {
      power *= -1;
    }

    double robotTurn = displacementInInches * ticksPerInch;

    runWithEncoder((int) robotTurn, (int) robotTurn, power, power);
  }

  @Override
  public void driveBackwards(double distanceInInches, double power) {
    driveWithEncoder(-Math.abs(distanceInInches), -Math.abs(power));
  }

  @Override
  public void rotateClockwise(int degrees, double givenPower) {
    if (degrees < 0)
      rotateCounterClockwise(-degrees, givenPower);
    else {
      double power = Range.clip(givenPower, -1, 1);
      power = Math.abs(power);
      double normalizedDegrees = Converter.normalizedDegrees(degrees);

      rotateWithEncoder(normalizedDegrees, -normalizedDegrees, power, -power);
    }
  }

  @Override
  public void rotateCounterClockwise(int degrees, double givenPower) {
    if (degrees < 0)
      rotateClockwise(-degrees, givenPower);
    else {
      double power = Range.clip(givenPower, -1, 1);
      power = Math.abs(power);
      double normalizedDegrees = Converter.normalizedDegrees(degrees);

      rotateWithEncoder(-normalizedDegrees, normalizedDegrees, -power, power);
    }
  }

  private void rotateWithEncoder(double leftDegrees, double rightDegrees,
                                 double leftPower, double rightPower) {

    runWithEncoder(
            (int) Math.round(leftDegrees / 360.0 * ticksPer360),
            (int) Math.round(rightDegrees / 360.0 * ticksPer360),
            leftPower, rightPower);
  }

  private void runWithEncoder(int leftTickOffset, int rightTickOffset,
                              double leftPower, double rightPower) {

    // Fails unit tests
    /*Log.d("TankDrive Encoder",
        String.format("leftTickOffset=%d rightTickOffset=%d leftPower=%.3f rightPower=%.3f",
        leftTickOffset, rightTickOffset, leftPower, rightPower));*/

    motorL.startRunToPosition(leftTickOffset, Math.abs(leftPower));
    motorR.startRunToPosition(rightTickOffset, Math.abs(rightPower));

    while (isBusy() && opModeIsActive()) {
    }

    stop();
    setMotorMode(RUN_USING_ENCODER);
  }

  // -- TeleOp methods --

  @Override
  public void setMovementPower(double power) {
    setMotorMode(RUN_WITHOUT_ENCODER);

    motorL.setPower(power);
    motorR.setPower(power);
  }

  @Override
  public void setRotationPower(double power) { //clockwise if power is positive
    setMotorMode(RUN_WITHOUT_ENCODER);

    motorL.setPower(power);
    motorR.setPower(-power);
  }

  @Override
  public void setMovementAndRotation(double movePower, double rotatePower) {
    double leftPower = movePower + rotatePower,
            rightPower = movePower - rotatePower,
            maxAbsPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));

    if (maxAbsPower > 1) {
      leftPower /= maxAbsPower;
      rightPower /= maxAbsPower;
    }

    motorL.setPower(leftPower);
    motorR.setPower(rightPower);
  }

  @Override
  protected IMotor[] getMotors() {
    return new IMotor[]{motorL, motorR};
  }

}
