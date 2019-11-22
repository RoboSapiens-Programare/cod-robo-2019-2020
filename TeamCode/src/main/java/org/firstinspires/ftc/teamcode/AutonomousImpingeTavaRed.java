package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomie Impinge Tava Red")
public final class AutonomousImpingeTavaRed extends AutonomousMain {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        // EXPERIMENTAL

        //Mers pana la 40 de cm de perete
        StrafeWithEncoders(0, 0.5, 200); RotateReset();
        RotateRelative(-90);
        sleep(100);
        StrafeToObstacle(30); //TODO

        //Ma rotesc ca sa am clestele orientat spre tava si merg pana la tava si o apuc
        RotateRelative(180);
        StrafeWithEncoders(-90, 0.5, 1000); RotateReset();
        ServoBrat.setPosition(0.1);
        sleep(200);

        //Trag tava si ma desprind si ma rotesc inapoi
        StrafeWithEncoders(90, 0.8, 1500);
        ServoBrat.setPosition(1); RotateReset();
        sleep(200);
        RotateRelative(180);

        //Ocoleste tava
        StrafeWithEncoders(180, 0.5, 500); RotateReset();
        StrafeToObstacle(90);
        StrafeWithEncoders(90 , 0.5 , 1000); RotateReset();
        StrafeWithEncoders(0 , 0.5 , 100); RotateReset();
        StrafeWithEncoders(90 , 0.5 , 1200); RotateReset();
        StrafeToObstacle(30);
        StrafeWithEncoders(-90 , 0.8 , 2000); RotateReset();

        //Parcheaza
        StrafeUntilColor(0.5, -180, ColorUnderneath, 9 ,10);


        //dupa ce trag tava, o ocolesc, intai la dreapta
        //apoi in fata, apoi la stanga, apoi in spate

        //dupa ce am impins tava, ma duc in dreapta
        // ma duc in spate pana la zid (senzori si poate ne rotim 180)
        //dupa mergem la dreapta pana la linie
    }
}
