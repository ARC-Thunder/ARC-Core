package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Lift Reset TeleOp", group = "ARC Thunder")
public class LiftReset extends OpMode {

    private DcMotor motorLatch;

    public void init() {
        motorLatch = hardwareMap.dcMotor.get("motorLatch");
    }

    public void loop() {
        double liftPower = 0;

        if (gamepad1.left_trigger >= 0.25)
            liftPower = -gamepad1.left_trigger;
        else if (gamepad1.right_trigger >= 0.25)
            liftPower = gamepad1.right_trigger;

        motorLatch.setPower(liftPower);
    }
}
