package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomie Impinge Tava Blue Perete")
public final class AutonomousImpingeTavaBluePerete extends AutonomousMain {
    final double DEADZONE = 0.1;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        AutonomousImpingeTavaBlue();

        StrafeWithEncoders(180, 0.4, 600); RotateReset();

    }
}
