package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous (Near Crater)", group = "Autonomous")
public class AutonomousCrater extends AutonomousMaster {
    @Override
    public void runOpMode() {
        try {
            super.runOpMode(); // To keep the setup code
            vuforia.stop();

            checkForInterrupt();

            // TODO: ADD CUSTOM AUTONOMOUS CODE HERE, USING checkForInterrupt TO CATCH INTERRUPTIONS

            stop();
        } catch (InterruptedException e) {
            asyncExecutor.shutdown();
            vuforia.stop();
            stop();
        }
    }
}
