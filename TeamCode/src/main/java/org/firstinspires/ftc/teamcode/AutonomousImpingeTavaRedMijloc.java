package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomie Impinge Tava Red Mijloc")
public final class AutonomousImpingeTavaRedMijloc extends AutonomousMain {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        AutonomousImpingeTavaRed();

        StrafeWithEncoders(180, 0.4, 600); RotateReset();
    }
}
