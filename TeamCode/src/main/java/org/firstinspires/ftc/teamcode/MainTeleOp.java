package org.firstinspires.ftc.teamcode;

import com.andoverrobotics.core.drivetrain.MecanumDrive;
import com.andoverrobotics.core.drivetrain.TankDrive;
import com.andoverrobotics.core.utilities.Coordinate;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

@TeleOp(name = "Main TeleOp", group = "ARC Thunder")
public class MainTeleOp extends OpMode {
    //The distance between the front wheels, the back wheels, and the front and the back wheels, in inches. Currently unset because measuring is hard.
    private static final double FRONT_WHEEL_DISTANCE = 14.8, BACK_WHEEL_DISTANCE = 14.8, FRONT_BACK_DISTANCE = 12.25, ROBOT_DIAMETER = 2 * Math.sqrt(Math.pow(1.0 / 2 * (FRONT_WHEEL_DISTANCE + BACK_WHEEL_DISTANCE) / 2, 2) + Math.pow(1 / 2 * FRONT_BACK_DISTANCE, 2));
    //TICKS_PER_WHEEL_360: how many ticks of a motor to make a wheel turn 360
    //ticksPer360: how many encoder ticks required to cause a full rotation for the robot, when this amount is applied to the left and right motors in opposite directions
    //ticksPer360 is currently calculated by multiplying ticksPerInch by the circumference of the circle with the rear axle as a diameter, as those are the wheels that are moving
    //ticksPerInch and ticksPer360 are rounded to the nearest integer
    private static final int WHEEL_DIAMETER_IN = 4, TICKS_PER_WHEEL = 1440, TICKS_PER_INCH = (int) (TICKS_PER_WHEEL / (Math.PI * WHEEL_DIAMETER_IN) + 0.5), TICKS_PER_360 = (int) (TICKS_PER_INCH * Math.PI * ROBOT_DIAMETER + 0.5);
    // KNOWN MOTOR TICKS (TICKS_PER_WHEEL_360):
    //     Tetrix DC Motors: 1440
    //     AndyMark NeveRest Motors: 1120 (Not 100% sure)
    private final double LIFT_HEIGHT_IN = 6.375;
    private final double PULLEY_DIAMETER_MM = 25;

    private int bucketMoveDelay = 350; // How long to wait before sending a new position to the bucket servos, in milliseconds
    private MecanumDrive mecanumDrive;
    private DcMotor motorLatch;

    private boolean aButton = false;

    protected Future<?> moveLatchMotor = null;
    protected ExecutorService asyncExecutor = Executors.newSingleThreadExecutor();

    public void init() {
        DcMotor motorFL = hardwareMap.dcMotor.get("motorFL");
        DcMotor motorFR = hardwareMap.dcMotor.get("motorFR");
        DcMotor motorBL = hardwareMap.dcMotor.get("motorBL");
        DcMotor motorBR = hardwareMap.dcMotor.get("motorBR");

        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);

        motorLatch = hardwareMap.dcMotor.get("motorLatch");
        motorLatch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mecanumDrive = MecanumDrive.fromCrossedMotors(motorFL, motorFR, motorBL, motorBR, this, TICKS_PER_INCH, TICKS_PER_360);
        mecanumDrive.setDefaultDrivePower(0.5);
    }

    public void loop() {
        double latchPower = 0;

        if (gamepad1.right_trigger >= 0.25)
            latchPower = -gamepad1.right_trigger;
        else if (gamepad1.left_trigger >= 0.25)
            latchPower = gamepad1.left_trigger;

        motorLatch.setPower(latchPower);

        if(gamepad1.a && !aButton)
            aButton = true;
        else if(gamepad1.a && aButton)
            aButton = false;

        raiseLatch((aButton) ? LIFT_HEIGHT_IN : 0, 0.5);

        if (Math.abs(gamepad1.right_stick_x) > 0.1) {
            mecanumDrive.setRotationPower(gamepad1.right_stick_x);
        } else {
            mecanumDrive.setStrafe(gamepad1.left_stick_x, -gamepad1.left_stick_y, 1);
        }
    }

    @Override
    public void stop() {
        super.stop();
    }

    protected void raiseLatch(final double inches, final double power) {
        while (moveLatchMotor != null && !moveLatchMotor.isDone()) {
        }

        double adjustedPower = Range.clip(-1, 1, power);
        adjustedPower *= (inches < 0) ? -1 : 1;

        final double endPower = adjustedPower;
        final DcMotor.RunMode oldRunMode = motorLatch.getMode();
        motorLatch.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Runnable moveLatch = new Runnable() {
            @Override
            public void run() {
                try {
                    motorLatch.setTargetPosition((int) (4 * 1440 * 25.4 / (Math.PI * PULLEY_DIAMETER_MM) * -inches + 0.5));
                    motorLatch.setPower(endPower);

                    while (motorLatch.isBusy()) {
                        checkForInterrupt();
                    }
                    motorLatch.setMode(oldRunMode);
                } catch (InterruptedException e) {
                    motorLatch.setTargetPosition(motorLatch.getCurrentPosition());
                    motorLatch.setPower(0);
                    motorLatch.setMode(oldRunMode);
                }
            }
        };
        moveLatchMotor = asyncExecutor.submit(moveLatch);
    }

    protected void checkForInterrupt() throws InterruptedException {
        if (Thread.interrupted())
            throw new InterruptedException();
    }
}