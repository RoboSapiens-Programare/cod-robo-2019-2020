package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomie Impinge Tava Red")
public final class AutonomousImpingeTavaRed extends AutonomousMain {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        //E fix la fel din punct de vedere al semnelor pt rotit si strafe:
        //I) Stanga si dreapta
        //Pt a roti robotul fie in stanga, fie in dreapta pe partea rosie la
        //acelasi unghi ca in blue, ar trebui pus un - unghiurilor din blue(1).
        //Faptul ca bratul ala mic se afla pe partea dreapta determina aparitia
        //a inca unui minus, data fiind noua pozitionare a robotului(2)
        //Din (1) si (2) => am angle*(-1)*(-1) = angle, unde angle = unghiul la
        //care m-as roti pe partea stanga
        //II) Fata si spate
        //Se vede ca-s la fel semnele aici I don't think i have to explain lmao
        //Singura probl e faptul ca nu avem senzori si pe spate

        // EXPERIMENTAL

        StrafeWithEncoders(0, 0.5, 200); RotateReset();

        RotateRelative(-90);

        sleep(100);

        StrafeToObstacle(20);

        RotateRelative(180);

        StrafeWithEncoders(-90, 0.5, 1300); RotateReset();

        ServoBrat.setPosition(0.1);

        sleep(200);

        StrafeWithEncoders(90, 0.8, 4000);

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
        StrafeWithEncoders(-90 , 0.8 , 5000); RotateReset();

        //Parcheaza
        StrafeUntilColor(0.5, -180, ColorUnderneath, 9 ,12);


        //dupa ce trag tava, o ocolesc, intai la dreapta
        //apoi in fata, apoi la stanga, apoi in spate

        //dupa ce am impins tava, ma duc in dreapta
        // ma duc in spate pana la zid (senzori si poate ne rotim 180)
        //dupa mergem la dreapta pana la linie
    }
}
