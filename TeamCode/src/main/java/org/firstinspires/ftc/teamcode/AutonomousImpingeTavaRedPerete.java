package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomie Impinge Tava Red Perete")
public final class AutonomousImpingeTavaRedPerete extends AutonomousMain {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        AutonomousImpingeTavaRed();

        StrafeWithEncoders(0, 0.4, 600); RotateReset();
    }
}
