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
                    telemetry.addData("Encoder Latch", motorLatch.getCurrentPosition());
                    telemetry.update();
                }
            }
        };

        Future<?> updateTelemetry = null;
        ExecutorService asyncExecutor = Executors.newSingleThreadExecutor();

        updateTelemetry = asyncExecutor.submit(encoderTelemetry);
        /*
        rotateLatchMotor(360, 0.5);
        sleep(1500);
        rotateLatchMotor(-360, 0.5);
        sleep(1500);
        */

        raiseLatch(3, 0.5);
        //sleep(10000);
        while(motorLatch.isBusy());
    }
}
