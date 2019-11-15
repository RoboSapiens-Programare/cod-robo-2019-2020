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

        StrafeWithEncoders(0, 0.5, 200);

        RotateRelative(90);

        //mers in fata pana la o dist de perete TODO

        sleep(100);

        StrafeToObstacle(25); RotateAbsolute(90);

        StrafeWithEncoders(-90, 0.5, 1100); RotateAbsolute(90);

        ServoBrat.setPosition(0.1);

        sleep(200);

        StrafeWithEncoders(90, 0.8, 4000);

        ServoBrat.setPosition(1);

        RotateAbsolute(90);

        sleep(200);

        //Ocoleste tava
        StrafeWithEncoders(-180 , 0.5 , 1600);
        RotateAbsolute(90);
        StrafeWithEncoders(-90 , 0.5 , 2500);
        RotateAbsolute(90);
        StrafeToObstacle(30);RotateAbsolute(90);
        StrafeWithEncoders(90 , 0.8 , 3500);
        RotateAbsolute(90);

        //Parcheaza
        StrafeUntilColor(0.5, -180, ColorUnderneath, 3);


        //dupa ce trag tava, o ocolesc, intai la dreapta
        //apoi in fata, apoi la stanga, apoi in spate

        //dupa ce am impins tava, ma duc in dreapta
        // ma duc in spate pana la zid (senzori si poate ne rotim 180)
        //dupa mergem la dreapta pana la linie

    }
}
