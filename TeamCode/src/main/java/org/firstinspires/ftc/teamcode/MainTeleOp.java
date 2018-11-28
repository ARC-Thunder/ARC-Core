package org.firstinspires.ftc.teamcode;

import com.andoverrobotics.core.drivetrain.TankDrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

@TeleOp(name = "Main TeleOp", group = "ARC Thunder")
public class MainTeleOp extends OpMode {
    //The distance between the front wheels, the back wheels, and the front and the back wheels, in inches. Currently unset because measuring is hard.
    private static final double FRONT_WHEEL_DISTANCE = 14.8, BACK_WHEEL_DISTANCE = 14.8, FRONT_BACK_DISTANCE = 12.25, ROBOT_DIAMETER = 2 * Math.sqrt(Math.pow(1/2 * (FRONT_WHEEL_DISTANCE + BACK_WHEEL_DISTANCE) / 2, 2) + Math.pow(1/2 * FRONT_BACK_DISTANCE, 2));
    //TICKS_PER_WHEEL_360: how many ticks of a motor to make a wheel turn 360
    //ticksPer360: how many encoder ticks required to cause a full rotation for the robot, when this amount is applied to the left and right motors in opposite directions
    //ticksPer360 is currently calculated by multiplying ticksPerInch by the circumference of the circle with the rear axle as a diameter, as those are the wheels that are moving
    //ticksPerInch and ticksPer360 are rounded to the nearest integer
    private static final int WHEEL_DIAMETER_IN = 4, TICKS_PER_WHEEL = 1440, TICKS_PER_INCH = (int) (TICKS_PER_WHEEL / (Math.PI * WHEEL_DIAMETER_IN) + 0.5), TICKS_PER_360 = (int) (TICKS_PER_INCH * Math.PI * ROBOT_DIAMETER + 0.5);
    // KNOWN MOTOR TICKS (TICKS_PER_WHEEL_360):
    //     Tetrix DC Motors: 1440
    //     AndyMark NeveRest Motors: 1120 (Not 100% sure)

    private TankDrive tankDrive;
    private DcMotor motorLift, motorThroat, motorIntakeL, motorIntakeR;

    private Servo servoBucketL, servoBucketR;
    public void init(){
        DcMotor motorFL = hardwareMap.dcMotor.get("motorFL");
        DcMotor motorFR = hardwareMap.dcMotor.get("motorFR");
        DcMotor motorBL = hardwareMap.dcMotor.get("motorBL");
        DcMotor motorBR = hardwareMap.dcMotor.get("motorBR");

        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);

        motorLift = hardwareMap.dcMotor.get("motorLift");
        motorThroat = hardwareMap.dcMotor.get("motorThroat");


//        motorIntakeL = hardwareMap.dcMotor.get("motorIntakeL");
//        motorIntakeR = hardwareMap.dcMotor.get("motorIntakeR");

//        motorIntakeL.setDirection(DcMotorSimple.Direction.REVERSE);

        servoBucketL = hardwareMap.servo.get("servoBucketL");
        servoBucketR = hardwareMap.servo.get("servoBucketR");

        servoBucketL.setDirection(Servo.Direction.REVERSE);

        tankDrive = TankDrive.fromMotors(motorFL, motorBL, motorFR, motorBR, this, TICKS_PER_INCH, TICKS_PER_360);
    }

    public void loop(){
        tankDrive.setMovementAndRotation(-gamepad1.left_stick_y, gamepad1.left_stick_x);

        motorLift.setPower(-gamepad2.left_stick_y);

        motorIntakeL.setPower(-gamepad2.right_stick_y);
        motorIntakeR.setPower(-gamepad2.right_stick_y);

        if(gamepad2.a){
            servoBucketL.setPosition(1);
            servoBucketR.setPosition(1);
        }
        else {
            servoBucketL.setPosition(0);
            servoBucketR.setPosition(0);
        }

        telemetry.addData("servoBucket Position:", servoBucketL.getPosition());
        telemetry.addData("motorLift Power:", motorLift.getPower());
        telemetry.addData("motorIntake Power:", motorIntakeL.getPower());

        telemetry.update();
    }
}
