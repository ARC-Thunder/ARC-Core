package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous (Near Depot)", group = "Autonomous")
public class AutonomousDepot extends AutonomousMaster {
    @Override
    public void runOpMode() {
        setup();

        mecanumDrive.driveForwards(25, 0.5);
        mecanumDrive.rotateClockwise(90, 0.554);
    }
}
