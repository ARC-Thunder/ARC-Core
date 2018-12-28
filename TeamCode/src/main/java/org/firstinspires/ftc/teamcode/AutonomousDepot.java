package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous (Near Depot)", group = "Autonomous")
public class AutonomousDepot extends AutonomousMaster {
    @Override
    public void runOpMode() {
        try {
            super.runOpMode(); // To keep the setup code
            vuforia.stop();

            // TODO: ADD CUSTOM AUTONOMOUS CODE HERE, USING checkForInterrupt TO CATCH INTERRUPTIONS
            stop();
        } catch (InterruptedException e) {
            vuforia.stop();
            stop();
        }
    }
}
