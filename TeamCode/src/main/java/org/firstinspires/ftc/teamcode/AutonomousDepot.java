package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;

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

        } catch (InterruptedException e) {
        }

    }
}
