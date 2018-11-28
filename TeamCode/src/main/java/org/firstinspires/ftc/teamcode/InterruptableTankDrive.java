package org.firstinspires.ftc.teamcode;

import com.andoverrobotics.core.drivetrain.TankDrive;
import com.andoverrobotics.core.utilities.Converter;
import com.andoverrobotics.core.utilities.IMotor;
import com.andoverrobotics.core.utilities.MotorAdapter;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Supplier;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

public class InterruptableTankDrive extends TankDrive {

    private IMotor motorL, motorR;
    private int ticksPerInch, ticksPer360;

    public InterruptableTankDrive(IMotor motorL, IMotor motorR, OpMode opMode, int ticksPerInch, int ticksPer360) {

        super(motorL, motorR, opMode, ticksPerInch, ticksPer360);

        this.motorL = motorL;
        this.motorR = motorR;
        this.ticksPer360 = ticksPer360;
        this.ticksPerInch = ticksPerInch;
    }

    public static InterruptableTankDrive fromMotors(DcMotor motorL, DcMotor motorR, OpMode opMode,
                                                    int ticksPerInch, int ticksPer360) {

        return new InterruptableTankDrive(new MotorAdapter(motorL),
                new MotorAdapter(motorR), opMode, ticksPerInch, ticksPer360);
    }

    public void rotateClockwise(int degrees, double givenPower, Supplier<Boolean> checkFunction) {
        if (degrees < 0)
            rotateCounterClockwise(-degrees, givenPower);
        else {
            double power = Range.clip(givenPower, -1, 1);
            power = Math.abs(power);
            double normalizedDegrees = Converter.normalizedDegrees(degrees);

            rotateWithEncoder(normalizedDegrees, -normalizedDegrees, power, -power, checkFunction);
        }
    }

    public void rotateCounterClockwise(int degrees, double givenPower, Supplier<Boolean> checkFunction) {
        if (degrees < 0)
            rotateClockwise(-degrees, givenPower);
        else {
            double power = Range.clip(givenPower, -1, 1);
            power = Math.abs(power);
            double normalizedDegrees = Converter.normalizedDegrees(degrees);

            rotateWithEncoder(-normalizedDegrees, normalizedDegrees, -power, power, checkFunction);
        }
    }

    private void rotateWithEncoder(double leftDegrees, double rightDegrees,
                                   double leftPower, double rightPower, Supplier<Boolean> checkFunction) {

        this.runWithEncoder(
                (int) Math.round(leftDegrees / 360.0 * ticksPer360),
                (int) Math.round(rightDegrees / 360.0 * ticksPer360),
                leftPower, rightPower, checkFunction);
    }

    private void runWithEncoder(int leftTickOffset, int rightTickOffset, double leftPower, double rightPower, Supplier<Boolean> checkFunction) {

        motorL.startRunToPosition(leftTickOffset, Math.abs(leftPower));
        motorR.startRunToPosition(rightTickOffset, Math.abs(rightPower));

        while (isBusy() && opModeIsActive()) {
            if (checkFunction.get()) {
                setMotorMode(STOP_AND_RESET_ENCODER);
                break;
            }
        }

        stop();
        setMotorMode(RUN_USING_ENCODER);
    }
}
