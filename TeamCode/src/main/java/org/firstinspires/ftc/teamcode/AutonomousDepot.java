package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

@Autonomous(name = "Autonomous (Near Depot)", group = "Autonomous")
public class AutonomousDepot extends AutonomousMaster {
    @Override
    public void runOpMode() {
        try {
            setup();
            super.runOpMode();

//            raiseLatch(LATCH_RAISE_DISTANCE, 0.5);
//
//            while (moveLatchMotor != null && !moveLatchMotor.isDone()) {
//                checkForInterrupt();
//            }


            //sleep(10000);
            //while(motorLatch.isBusy());
        } catch (InterruptedException e) {
            asyncExecutor.shutdown();
            vuforia.stop();
            stop();
        }
    }
}
