package org.firstinspires.ftc.teamcode;

import com.andoverrobotics.core.drivetrain.TankDrive;
import com.disnodeteam.dogecv.Dogeforia;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.detectgold.GoldDetection;

@Autonomous(name = "Main Autonomous", group = "Autonomous")
public class MainAutonomous extends LinearOpMode {
    //The distance between the front wheels, the back wheels, and the front and the back wheels, in inches.
    private static final double FRONT_WHEEL_DISTANCE = 14.8, BACK_WHEEL_DISTANCE = 14.8, FRONT_BACK_DISTANCE = 12.75, ROBOT_DIAMETER = 2 * Math.sqrt(Math.pow(1 / 2 * (FRONT_WHEEL_DISTANCE + BACK_WHEEL_DISTANCE) / 2, 2) + Math.pow(1 / 2 * FRONT_BACK_DISTANCE, 2));
    private final double CAMERA_HEIGHT = 5.25; // How high off the ground the phone's camera is, in inches
    // On the test bot, this is 5.2 inches if the camera is at the bottom or 9 inches if the camera is at the top

    //TICKS_PER_WHEEL_360: how many ticks of a motor to make a wheel turn 360
    //ticksPer360: how many encoder ticks required to cause a full rotation for the robot, when this amount is applied to the left and right motors in opposite directions
    //ticksPer360 is currently calculated by multiplying ticksPerInch by the circumference of the circle with the rear axle as a diameter, as those are the wheels that are moving
    //ticksPerInch and ticksPer360 are rounded to the nearest integer
    private static final int WHEEL_DIAMETER_IN = 4, TICKS_PER_WHEEL = 1440, TICKS_PER_INCH = (int) (TICKS_PER_WHEEL / (Math.PI * WHEEL_DIAMETER_IN) + 0.5), TICKS_PER_360 = (int) (TICKS_PER_INCH * Math.PI * 10.55 + 0.5);
    // KNOWN MOTOR TICKS (TICKS_PER_WHEEL_360):
    //     Tetrix DC Motors: 1440
    //     AndyMark NeveRest Motors: 1120 (Not 100% sure)

    private final double CAM_FOCAL_LENGTH = 751.0, GOLD_WIDTH_IN = 2; // Approximate focal length of a Moto G (2nd gen): 637.5 Old focal length: 560
    private GoldDetection goldDetection;

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = VuforiaLocalizer.CameraDirection.BACK;

    private final String VUFORIA_KEY = "AQRacK7/////AAAAGea1bsBsYEJvq6S3KuXK4PYTz4IZmGA7SV88bdM7l26beSEWkZTUb8H352Bo/ZMC6krwmfEuXiK7d7qdFkeBt8BaD0TZAYBMwHoBkb7IBgMuDF4fnx2KiQPOvwBdsIYSIFjiJgGlSj8pKZI+M5qiLb3DG3Ty884EmsqWQY0gjd6RNhtSR+6oiXazLhezm9msyHWZtX5hQFd9XoG5npm4HoGaZNdB3g5YCAQNHipjTm3Vkf71rG/Fffif8UTCI1frmKYtb4RvqiixDSPrD6OG6YmbsPOYUt2RZ6sSTreMzVL76CNfBTzmpo2V0E6KKP2y9N19hAum3GZu3G/1GEB5D+ckL/CXk4JM66sJw3PGucCs";
    private final double MAX_TRAVEL = Math.sqrt(Math.pow(24, 2) + Math.pow(24, 2));

    private DcMotor motorFL, motorFR, motorBL, motorBR, motorLift, motorThroat;
    private Servo servoBucketL, servoBucketR;
    private CRServo crServoIntakeL, crServoIntakeR;
    private TankDrive tankDrive;
    private Dogeforia vuforia;

    @Override
    public void runOpMode() {
        try {
            setup();
            checkForInterrupt();

            while (!opModeIsActive()) {
                checkForInterrupt();
            }

            //tankDrive.driveForwards(Math.sqrt(Math.pow(13.5, 2)) - 8.45, 0.5);

            Accelerometer accelerometer = new Accelerometer(hardwareMap);

            motorThroat.setPower(-0.75); // Lower the intake system

            while (accelerometer.getZ() > -9) {
                checkForInterrupt();

                // Wait for the phone to start to reach the top of the drop\
                telemetry.addData("Z Value", accelerometer.getZ());
                telemetry.update();
            }

            telemetry.addLine("Successfully stopped motor");
            telemetry.update();

            Thread.sleep(375); // Make sure it starts to drop before cutting the power/

            motorThroat.setPower(0);
            accelerometer.stop();

            checkForInterrupt();

            Thread.sleep(1000);

            goldDetection = new GoldDetection(CAM_FOCAL_LENGTH, GOLD_WIDTH_IN, MAX_TRAVEL, CAMERA_HEIGHT, hardwareMap, vuforia);

            double[] goldOffset = goldDetection.getGoldOffset(); // Format: [distanceToTravel, roundedAngle]
            double distanceToTravel = goldOffset[0];
            int roundedAngle = (int) (goldOffset[1]);

            checkForInterrupt();

            telemetry.addData("DistanceToTravel", distanceToTravel);
            telemetry.addData("RoundedAngle", roundedAngle);

            telemetry.update();

            crServoIntakeL.setPower(1);
            crServoIntakeR.setPower(1);
            checkForInterrupt();

            tankDrive.rotateClockwise(roundedAngle, 0.5);
            tankDrive.driveForwards(distanceToTravel, 0.5);
            checkForInterrupt();

            Thread.sleep(5000); // Give the cube 5 seconds to be taken in
            crServoIntakeL.setPower(0);
            crServoIntakeR.setPower(0);

            checkForInterrupt();

            tankDrive.driveBackwards(distanceToTravel, 0.5);
            tankDrive.rotateClockwise(-roundedAngle, 0.5);
            checkForInterrupt();

            vuforia.stop();
            stop();
        } catch (InterruptedException e) {
            vuforia.stop();
            stop();
        }
    }

    private void setup() {
        telemetry.addData("Status", "Main Autonomous");

        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");

        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);

        motorLift = hardwareMap.dcMotor.get("motorLift");
        motorThroat = hardwareMap.dcMotor.get("motorThroat");

        crServoIntakeL = hardwareMap.crservo.get("crServoIntakeL");
        crServoIntakeR = hardwareMap.crservo.get("crServoIntakeR");

        crServoIntakeR.setDirection(CRServo.Direction.REVERSE);

        servoBucketL = hardwareMap.servo.get("servoBucketL");
        servoBucketR = hardwareMap.servo.get("servoBucketR");

        servoBucketL.setDirection(Servo.Direction.REVERSE);

        tankDrive = TankDrive.fromMotors(motorFL, motorBL, motorFR, motorBR, this, TICKS_PER_INCH, TICKS_PER_360);

        // Set up DogeCV and Dogeforia
        Dogeforia.Parameters parameters = new Dogeforia.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        parameters.cameraDirection = CAMERA_CHOICE;

        parameters.fillCameraMonitorViewParent = true;

        vuforia = new Dogeforia(parameters);

        vuforia.enableConvertFrameToBitmap();

        waitForStart();
    }

    private void depositTeamMarker() {
        motorLift.setPower(0.75);
        sleep(1000);
        servoBucketL.setPosition(0.5);
        servoBucketR.setPosition(0.5);
        sleep(1000);
        motorLift.setPower(-0.75);
        sleep(1000);
        servoBucketL.setPosition(0.75);
        servoBucketR.setPosition(0.75);
        sleep(750);
        servoBucketL.setPosition(1);
        servoBucketR.setPosition(1);
        sleep(750);
        motorLift.setPower(0.75);
        sleep(1000);
        servoBucketR.setPosition(0);
        servoBucketL.setPosition(0);
        sleep(1000);
        motorLift.setPower(-0.75);
        sleep(1000);
    }

    private void checkForInterrupt() throws InterruptedException {
        if(Thread.interrupted())
            throw new InterruptedException();
    }
}
