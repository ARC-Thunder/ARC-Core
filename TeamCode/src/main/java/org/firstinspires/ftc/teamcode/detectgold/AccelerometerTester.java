package org.firstinspires.ftc.teamcode.detectgold;

import org.firstinspires.ftc.teamcode.detectgold.Accelerometer.PhoneRotation;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Accelerometer Tester")
public class AccelerometerTester extends LinearOpMode {


    private Accelerometer accelerometer;

    @Override
    public void runOpMode() {
        setup();

        while (opModeIsActive()) {
            PhoneRotation phoneRotation = accelerometer.getPhoneRotation();

            telemetry.addData("X", accelerometer.getX());
            telemetry.addData("Y", accelerometer.getY());
            telemetry.addData("Z", accelerometer.getZ());
            telemetry.addData("Phone Rotation", phoneRotation.toString());
            telemetry.update();
        }

        stopSensors();
    }

    private void setup() {
        accelerometer = new Accelerometer(hardwareMap);

        waitForStart();
    }

    private void stopSensors() {
        accelerometer.stop();
    }

}
