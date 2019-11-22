package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomie Sky Stone Blue Mijloc")
public final class AutonomousSkyStoneBlueMijloc extends AutonomousMain {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        AutonomousSkyStoneBlue();

        StrafeWithEncoders(-90, 0.4, 600); RotateReset();

    }
}
