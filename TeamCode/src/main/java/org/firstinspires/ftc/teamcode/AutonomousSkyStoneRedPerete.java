package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomie Sky Stone Red Perete")
public final class AutonomousSkyStoneRedPerete extends AutonomousMain {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        AutonomousSkyStoneRed();

        StrafeWithEncoders(180, 0.4, 400); RotateReset();

    }
}
