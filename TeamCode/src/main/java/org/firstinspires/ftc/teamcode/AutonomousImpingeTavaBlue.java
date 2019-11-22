package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomie Impinge Tava Blue")
public final class AutonomousImpingeTavaBlue extends AutonomousMain {
    final double DEADZONE = 0.1;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        // EXPERIMENTAL

        //Merg pana la 40 de cm de perete
        StrafeWithEncoders(0, 0.5, 200); RotateReset();
        RotateRelative(90);
        sleep(100);
        StrafeToObstacle(40); //TODO

        //Merg pana la tava si o apuc
        StrafeWithEncoders(-90, 0.5, 1100); RotateReset();
        ServoBrat.setPosition(0.1);
        sleep(200);

        //Trag tava si ma desprind
        StrafeWithEncoders(90, 0.6, 2500);
        ServoBrat.setPosition(1); RotateReset();
        sleep(200);

        //Ocolesc tava
        StrafeToObstacle(100);
        StrafeWithEncoders(-90 , 0.5 , 2200); RotateReset();
        StrafeToObstacle(30);
        StrafeWithEncoders(90 , 0.8 , 2500); RotateReset();

        //Parchez
        StrafeUntilColor(0.5, -180, ColorUnderneath, 2 , 4);


        //dupa ce trag tava, o ocolesc, intai la dreapta
        //apoi in fata, apoi la stanga, apoi in spate

        //dupa ce am impins tava, ma duc in dreapta
        // ma duc in spate pana la zid (senzori si poate ne rotim 180)
        //dupa mergem la dreapta pana la linie

    }
}
