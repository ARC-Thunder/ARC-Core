package org.firstinspires.ftc.teamcode;

import com.andoverrobotics.core.drivetrain.TankDrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    private int bucketMoveDelay = 350; // How long to wait before sending a new position to the bucket servos, in milliseconds

    private TankDrive tankDrive;
    private DcMotor motorLift, motorThroat;

//    private Runnable dumpBucket = new Runnable() {
//        @Override
//        public void run() {
//            for (int i = 4; i >= 0; i--) {
//                setServoBucketsPosition(0.25 * i);
//                try {
//                    Thread.sleep(bucketMoveDelay);
//                } catch (InterruptedException e) {
//                }
//            }
//
//            try {
//                Thread.sleep(bucketMoveDelay);
//            } catch (InterruptedException e) {
//            }
//        }
//    };

    private Future<?> depositTeamMarkerResult = null;
    private ExecutorService asyncExecutor = Executors.newSingleThreadExecutor();

    public void init() {
        DcMotor motorFL = hardwareMap.dcMotor.get("motorFL");
        DcMotor motorFR = hardwareMap.dcMotor.get("motorFR");
        DcMotor motorBL = hardwareMap.dcMotor.get("motorBL");
        DcMotor motorBR = hardwareMap.dcMotor.get("motorBR");

        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);

        tankDrive = TankDrive.fromMotors(motorFL, motorBL, motorFR, motorBR, this, TICKS_PER_INCH, TICKS_PER_360);
    }

    public void loop() {
        double liftPower = 0, throatPower = 0;

        if (gamepad1.right_trigger >= 0.25)
            liftPower = 1;
        else if (gamepad1.left_trigger >= 0.25)
            liftPower = -1;

        motorLift.setPower(liftPower);

        if (gamepad1.right_bumper)
            throatPower = 1;
        else if (gamepad1.left_bumper)
            throatPower = -1;

        motorThroat.setPower(throatPower);

        tankDrive.setMovementAndRotation(-gamepad1.left_stick_y, gamepad1.left_stick_x);

//        if (depositTeamMarkerResult == null || depositTeamMarkerResult.isDone()) {
//            if (gamepad1.a && servoBucketL.getPosition() > 0 && servoBucketR.getPosition() > 0)
//                depositTeamMarkerResult = asyncExecutor.submit(dumpBucket);
//            else if (!gamepad1.a && servoBucketL.getPosition() < 1 && servoBucketR.getPosition() < 1)
//                depositTeamMarkerResult = asyncExecutor.submit(lowerBucket);
//        }
    }

    @Override
    public void stop() {
        super.stop();
        asyncExecutor.shutdown();
    }

}