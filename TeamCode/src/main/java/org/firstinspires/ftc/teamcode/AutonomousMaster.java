package org.firstinspires.ftc.teamcode;

import com.andoverrobotics.core.drivetrain.MecanumDrive;
import com.andoverrobotics.core.utilities.Converter;
import com.disnodeteam.dogecv.Dogeforia;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.detectgold.GoldDetection;

public class AutonomousMaster extends LinearOpMode {
    //The distance between the front wheels, the back wheels, and the front and the back wheels, in inches.
    protected static final double FRONT_WHEEL_DISTANCE = 14.8, BACK_WHEEL_DISTANCE = 14.8, FRONT_BACK_DISTANCE = 12.75, ROBOT_DIAMETER = 2 * Math.sqrt(Math.pow(0.5 * (FRONT_WHEEL_DISTANCE + BACK_WHEEL_DISTANCE) / 2, 2) + Math.pow(0.5 * FRONT_BACK_DISTANCE, 2));
    //TICKS_PER_WHEEL_360: how many ticks of a motor to make a wheel turn 360
    //ticksPer360: how many encoder ticks required to cause a full rotation for the robot, when this amount is applied to the left and right motors in opposite directions
    //ticksPer360 is currently calculated by multiplying ticksPerInch by the circumference of the circle with the rear axle as a diameter, as those are the wheels that are moving
    //ticksPerInch and ticksPer360 are rounded to the nearest integer
    protected static final int WHEEL_DIAMETER_IN = (int)(Converter.millimetersToInches(100) + 0.5), TICKS_PER_WHEEL = 1331, TICKS_PER_INCH = (int) (TICKS_PER_WHEEL / (Math.PI * WHEEL_DIAMETER_IN) + 0.5), TICKS_PER_360 = (int) (TICKS_PER_INCH * Math.PI * ROBOT_DIAMETER + 0.5);
    // On the test bot, this is 5.2 inches if the camera is at the bottom or 9 inches if the camera is at the top
    // KNOWN MOTOR TICKS (TICKS_PER_WHEEL_360):
    //     Tetrix DC Motors: 1440
    //     AndyMark NeveRest Motors: 1120 (Not 100% sure )
    protected static final double CAMERA_DISTANCE_FROM_FRONT = 8.5;
    protected static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = VuforiaLocalizer.CameraDirection.BACK;
    protected final double CAMERA_HEIGHT = 7.5; // How high off the ground the phone's camera is, in inches
    protected final double CAM_FOCAL_LENGTH = 751.0, GOLD_WIDTH_IN = 2; // Approximate focal length of a Moto G (2nd gen): 637.5 Old focal length: 560
    protected final String VUFORIA_KEY = "AQRacK7/////AAAAGea1bsBsYEJvq6S3KuXK4PYTz4IZmGA7SV88bdM7l26beSEWkZTUb8H352Bo/ZMC6krwmfEuXiK7d7qdFkeBt8BaD0TZAYBMwHoBkb7IBgMuDF4fnx2KiQPOvwBdsIYSIFjiJgGlSj8pKZI+M5qiLb3DG3Ty884EmsqWQY0gjd6RNhtSR+6oiXazLhezm9msyHWZtX5hQFd9XoG5npm4HoGaZNdB3g5YCAQNHipjTm3Vkf71rG/Fffif8UTCI1frmKYtb4RvqiixDSPrD6OG6YmbsPOYUt2RZ6sSTreMzVL76CNfBTzmpo2V0E6KKP2y9N19hAum3GZu3G/1GEB5D+ckL/CXk4JM66sJw3PGucCs";
    protected final double MAX_TRAVEL = Math.sqrt(Math.pow(24, 2) + Math.pow(24, 2));
    protected GoldDetection goldDetection;
    protected DcMotor motorFL, motorFR, motorBL, motorBR;
    protected MecanumDrive mecanumDrive;
    protected Dogeforia vuforia;

    /**
     * Sets up mecanumDrive and vuforia, uses GoldDetection to detect and collect the gold
     */
    @Override
    public void runOpMode() throws InterruptedException {
        setup();
        checkForInterrupt();

        while (!opModeIsActive()) {
            checkForInterrupt();
        }

        //mecanumDrive.driveForwards(Math.sqrt(Math.pow(13.5, 2)) - 8.45, 0.5);

        checkForInterrupt();

        Thread.sleep(1000);

        goldDetection = new GoldDetection(CAM_FOCAL_LENGTH, GOLD_WIDTH_IN, MAX_TRAVEL, CAMERA_HEIGHT, CAMERA_DISTANCE_FROM_FRONT, hardwareMap, vuforia);

        double[] goldOffset = goldDetection.getGoldOffset(); // Format: [distanceToTravel, roundedAngle]
        if (goldOffset[0] == Double.MIN_VALUE || goldOffset[1] == Double.MIN_VALUE) {
            throw new InterruptedException();
        }
        double distanceToTravel = goldOffset[0];
        int roundedAngle = (int) (goldOffset[1]);

        checkForInterrupt();

        telemetry.addData("DistanceToTravel", distanceToTravel);
        telemetry.addData("RoundedAngle", roundedAngle);

        telemetry.update();

        checkForInterrupt();

        mecanumDrive.rotateClockwise(roundedAngle, 0.5);
        mecanumDrive.driveForwards(distanceToTravel, 0.5);
        checkForInterrupt();

        Thread.sleep(5000); // Give the cube 5 seconds to be taken in

        checkForInterrupt();

        mecanumDrive.driveBackwards(distanceToTravel, 0.5);
        mecanumDrive.rotateClockwise(-roundedAngle, 0.5);
        checkForInterrupt();
    }

    protected void setup() {
        telemetry.addData("Status", "Main Autonomous");

        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");

        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);

        mecanumDrive = MecanumDrive.fromOctagonalMotors(motorFL, motorBL, motorFR, motorBR, this, TICKS_PER_INCH, TICKS_PER_360);

        // Set up DogeCV and Dogeforia
        Dogeforia.Parameters parameters = new Dogeforia.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        parameters.cameraDirection = CAMERA_CHOICE;

        parameters.fillCameraMonitorViewParent = true;

        vuforia = new Dogeforia(parameters);

        vuforia.enableConvertFrameToBitmap();

        waitForStartWithPings();
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
}
