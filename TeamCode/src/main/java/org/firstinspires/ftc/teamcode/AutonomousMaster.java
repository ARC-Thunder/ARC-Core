package org.firstinspires.ftc.teamcode;

import com.andoverrobotics.core.drivetrain.MecanumDrive;
import com.andoverrobotics.core.utilities.Converter;
import com.disnodeteam.dogecv.Dogeforia;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.detectgold.GoldAlignDetection;
import org.firstinspires.ftc.teamcode.detectgold.VuforiaGoldAlignDetection;
import org.firstinspires.ftc.teamcode.detectgold.GoldDetection;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

public class AutonomousMaster extends LinearOpMode {
    //The distance between the front wheels, the back wheels, and the front and the back wheels, in inches.
    protected static final double FRONT_WHEEL_DISTANCE = 15.125, BACK_WHEEL_DISTANCE = 15.125, FRONT_BACK_DISTANCE = 12.75, ROBOT_DIAMETER = 2 * Math.sqrt(Math.pow(0.5 * (FRONT_WHEEL_DISTANCE + BACK_WHEEL_DISTANCE) / 2, 2) + Math.pow(0.5 * FRONT_BACK_DISTANCE, 2));
    //TICKS_PER_WHEEL_360: how many ticks of a motor to make a wheel turn 360
    //ticksPer360: how many encoder ticks required to cause a full rotation for the robot, when this amount is applied to the left and right motors in opposite directions
    //ticksPer360 is currently calculated by multiplying ticksPerInch by the circumference of the circle with the rear axle as a diameter, as those are the wheels that are moving
    //ticksPerInch and ticksPer360 are rounded to the nearest integer
    protected static final int WHEEL_DIAMETER_IN = (int) (Converter.millimetersToInches(100) + 0.5), TICKS_PER_WHEEL = 1570, TICKS_PER_INCH = (int) (TICKS_PER_WHEEL / (Math.PI * WHEEL_DIAMETER_IN) + 0.5), TICKS_PER_360 = (int) (TICKS_PER_INCH * Math.PI * ROBOT_DIAMETER + 0.5);
    // On the test bot, this is 5.2 inches if the camera is at the bottom or 9 inches if the camera is at the top
    // KNOWN MOTOR TICKS (TICKS_PER_WHEEL_360):
    //     Tetrix DC Motors: 1440
    //     AndyMark NeveRest Motors: 1120 (Not 100% sure )
    protected static final double CAMERA_DISTANCE_FROM_FRONT = 0;
    protected static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = VuforiaLocalizer.CameraDirection.FRONT;
    protected final double CAMERA_HEIGHT = 3.75; // How high off the ground the phone's camera is, in inches
    protected final double CAM_FOCAL_LENGTH = 751.0, GOLD_WIDTH_IN = 2; // Approximate focal length of a Moto G (2nd gen): 637.5 Old focal length: 560
    protected final String VUFORIA_KEY = "ATQZG9T/////AAABmVE44Js1eEitobSAT11TPWgVWpehtY2ffxf0OR4SebS7RHP+3yzoO+VrtWVCLxMYUmSBmrrU6wgXGe+ngM0D40IuvC2yHn4XxbrTTWY3l1/LU1XizPh5DnJ+m08z/VKW47kIC165vOgHef7HXSaJkWZNG0ovW3UTfTXOjA3YOvso2EPYP9gFmi9a2ak0VB6iqew9WpfVCKCX8ehTNa9duNSuCmodIqWTc+S90/VgPVo086NlcecvEadyJjz6U8YZK/o9VRPh2sQ9SlXApo3y2m9dZbiwBvfs5a0GVLBzdzwUh/0hLhUOZZndJY5+6CgGsL/0yEukEAFpMslapJGDARdSa7eRvYqnOCZsjMmxbRE6";
    protected final double MAX_TRAVEL = Math.sqrt(Math.pow(24, 2) + Math.pow(24, 2));
    protected GoldDetection goldDetection;
    protected DcMotor motorFL, motorFR, motorBL, motorBR, motorLatch;
    protected MecanumDrive mecanumDrive;
    protected Dogeforia vuforia;
    protected final double PULLEY_DIAMETER_MM = 25;
    protected final double LATCH_RAISE_DISTANCE = 5 + 0.75 / 2; // How far up to move the latch lift to hook, from a position flush with the plate underneath the 80-20

    private final double DISTANCE_BETWEEN_MINERALS = 14.5; // How far in between the minerals, in inches
    private final double DISTANCE_TO_MINERALS = 1.25 * Math.sqrt(Math.pow(18, 2) + Math.pow(14, 2)); // How far from the robot's scanning point to the minerals, in inches

    protected GoldAlignDetection goldAlignDetection;
    protected VuforiaGoldAlignDetection vuforiaGoldAlignDetection;
    protected Future<?> moveLatchMotor = null;
    protected ExecutorService asyncExecutor = Executors.newSingleThreadExecutor();

    /**
     * Sets up mecanumDrive and vuforia, uses GoldDetection to detect and collect the gold
     */
    @Override
    public void runOpMode() {
        try {
            setup();
            checkForInterrupt();

            waitForStartWithPings();

//            vuforiaGoldAlignDetection = new VuforiaGoldAlignDetection(hardwareMap, VUFORIA_KEY, CAMERA_CHOICE, CAMERA_CHOICE == VuforiaLocalizer.CameraDirection.BACK ? 0 : 1);
//            vuforia = vuforiaGoldAlignDetection.getVuforia();

//            raiseLatch(LATCH_RAISE_DISTANCE, 0.5);
//            while (!moveLatchMotor.isDone()) {
//                checkForInterrupt();
//            }
//
//            mecanumDrive.strafeLeft(5, 0.75);
//            mecanumDrive.driveForwards(8, 0.75);
//            mecanumDrive.strafeRight(5, 0.75);
//
//            mecanumDrive.rotateClockwise(45, 0.75);
            //hitGoldRotate();
            hitGoldStrafe();
            //vuforiaGoldAlignDetection.disable();
            //vuforia.stop();
            goldAlignDetection.disable();

        } catch (InterruptedException e) {
            //vuforiaGoldAlignDetection.disable();
            //vuforia.stop();
            goldAlignDetection.disable();
        }
//        } catch (VuforiaException e) {
//            vuforiaGoldAlignDetection.disable();
//            vuforia.stop();
//        }
    }

    protected void setup() {
        telemetry.addData("Status", "Main Autonomous");

        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorLatch = hardwareMap.dcMotor.get("motorLatch");

        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLatch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mecanumDrive = MecanumDrive.fromCrossedMotors(motorFL, motorFR, motorBL, motorBR, this, TICKS_PER_INCH, TICKS_PER_360);
        mecanumDrive.setDefaultDrivePower(0.5);

        goldAlignDetection = new GoldAlignDetection(hardwareMap, CAMERA_CHOICE == VuforiaLocalizer.CameraDirection.BACK ? 0 : 1);
    }

    private void waitForStartWithPings() {
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Waiting in Init", System.currentTimeMillis());
            telemetry.update();
        }
    }

    protected void checkForInterrupt() throws InterruptedException {
        if (Thread.interrupted())
            throw new InterruptedException();
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
                    motorLatch.setTargetPosition((int) (4 * 1680 * 25.4 / (Math.PI * PULLEY_DIAMETER_MM) * -inches + 0.5));
                    motorLatch.setPower(endPower);

                    while (motorLatch.isBusy()) {
                        telemetry.addData("encoder position", motorLatch.getCurrentPosition());
                        telemetry.addData("encoder target", motorLatch.getTargetPosition());
                        telemetry.update();
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

    // Collects the gold by strafing between minerals and checking if gold is aligned with the robot

    private void hitGoldStrafe() {
        double distanceStrafed = 0; // How far the robot has strafed (in the positive x direction)

        telemetry.addData("Aligned", goldAlignDetection.isAligned());
        telemetry.update();

        if (goldAlignDetection.isAligned())
            mecanumDrive.driveForwards(DISTANCE_TO_MINERALS, 0.5);
        else {
            mecanumDrive.strafeInches(DISTANCE_BETWEEN_MINERALS, 0, 0.5);
            distanceStrafed = DISTANCE_BETWEEN_MINERALS;

            sleep(1000);

            telemetry.addData("Aligned", goldAlignDetection.isAligned());
            telemetry.update();

            if (goldAlignDetection.isAligned())
                mecanumDrive.driveForwards(DISTANCE_TO_MINERALS, 0.5);
            else {
                mecanumDrive.strafeInches(-2 * DISTANCE_BETWEEN_MINERALS, 0, 0.5);
                sleep(1000);

                if (goldAlignDetection.isAligned()) {
                    distanceStrafed = -DISTANCE_BETWEEN_MINERALS;
                    mecanumDrive.driveForwards(DISTANCE_TO_MINERALS, 0.5);
                }
            }
        }

        mecanumDrive.driveBackwards(DISTANCE_TO_MINERALS, 0.5);
        mecanumDrive.strafeInches(-distanceStrafed, 0, 0.5); // Go back to the starting point
    }

    // Hits the gold by rotating slowly until it is aligned
    private void hitGoldRotate() {
        mecanumDrive.setRotationPower(-0.125);
        mecanumDrive.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        try {
            while (!vuforiaGoldAlignDetection.isAligned()) {
                checkForInterrupt();
            }
        } catch (InterruptedException e) {
            mecanumDrive.stop();
        }

        mecanumDrive.stop();

        mecanumDrive.driveForwards(DISTANCE_TO_MINERALS, 0.5);
        mecanumDrive.driveBackwards(DISTANCE_TO_MINERALS, 0.5);
    }
}
