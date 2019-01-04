package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

@Autonomous(name = "Autonomous (Near Depot)", group = "Autonomous")
public class AutonomousDepot extends AutonomousMaster {
    @Override
    public void runOpMode() {
        setup();

        Runnable encoderTelemetry = new Runnable() {
            @Override
            public void run() {
                while (true) {
                    telemetry.addData("Encoder BL", motorBL.getCurrentPosition());
                    telemetry.addData("Encoder BR", motorBR.getCurrentPosition());
                    telemetry.addData("Encoder FL", motorFL.getCurrentPosition());
                    telemetry.addData("Encoder FR", motorBR.getCurrentPosition());
                    telemetry.update();
                }
            }
        };

        Future<?> updateTelemetry = null;
        ExecutorService asyncExecutor = Executors.newSingleThreadExecutor();

        updateTelemetry = asyncExecutor.submit(encoderTelemetry);

        mecanumDrive.driveForwards(24, 0.5);
        mecanumDrive.rotateClockwise(90, 0.5);
    }
}
