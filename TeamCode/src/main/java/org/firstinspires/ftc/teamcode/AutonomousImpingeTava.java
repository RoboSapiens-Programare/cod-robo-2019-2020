package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomie Impinge Tava")
public final class AutonomousImpingeTava extends AutonomousMain {
    final double DEADZONE = 0.1;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();


        StrafeWithEncoders(90, 0.4, 2000);
        StrafeWithEncoders(0, 0.4, 1050);

        Rotate(90);

        ServoBrat.setPosition(0.1); //1 si 0.1

        sleep(500);

        StrafeWithEncoders(90, 0.6, 900);

    }
}
