package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomie Impinge Tava")
public final class AutonomousImpingeTava extends AutonomousMain {
    final double DEADZONE = 0.1;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        // EXPERIMENTAL
        StrafeWithEncoders(45, 0.4, 2260);

        //StrafeWithEncoders(90, 0.4, 2000);
        //StrafeWithEncoders(0, 0.4, 1050);

        Rotate(90);

        ServoBrat.setPosition(0.1); //1 si 0.1

        sleep(500);

        StrafeWithEncoders(90, 0.8, 1800);

        //dupa ce trag tava, o ocolesc, intai la dreapta
        //apoi in fata, apoi la stanga, apoi in spate

        //dupa ce am impins tava, ma duc in dreapta
        // ma duc in spate pana la zid (senzori si poate ne rotim 180)
        //dupa mergem la dreapta pana la linie

    }
}
