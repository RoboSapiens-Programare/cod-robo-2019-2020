package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomie Sky Stone Blue Perete")
public final class AutonomousSkyStoneBluePerete extends AutonomousMain {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        AutonomousSkyStoneBlue();

        StrafeWithEncoders(90, 0.4, 600); RotateReset();

    }
}
