package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomie Sky Stone Red") //TODO momentan e copiat dupa blue
public final class AutonomousSkyStoneRed extends AutonomousMain {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        StrafeWithEncoders(-90 , 0.5 , 2000); RotateReset();

        //Mergem in fata si rotim sa fim cu senzorul inspre caramizi
        StrafeWithEncoders(0, 0.5, 300); RotateReset();
        RotateRelative(90);

        //Mergem pana gasim ceva de culoarea galbena
        StrafeUntilColor(0.15, -90, ColorSide, 1 , 16); RotateReset();


        sleep(200);

        //Incepem sa le luam in ordine pana ajungem la unul negru
        StrafeUntilColor(0.15, 0, ColorSide, 9 , 16); RotateReset();
        StrafeWithEncoders(0, 0.2, 50); RotateReset();

        sleep(200);

        //Dam bratul in jos
        ServoBrat.setPosition(0.1);

        //Tragem ala in building zone
        StrafeWithEncoders(90, 0.6, 1500); RotateReset();
        StrafeUntilColor(0.5, 0, ColorUnderneath, 2 , 4); RotateReset();

        //Ridicam bratul
        RotateRelative(90);
        sleep(300);
        ServoBrat.setPosition(1);
        sleep(300);
        RotateRelative(90);

        StrafeToObstacle(100);

        //Mergem in fata si rotim sa fim cu senzorul inspre caramizi
        RotateRelative(180);

        //Mergem pana gasim ceva de culoarea galbena
        StrafeUntilColor(0.15, -90, ColorSide, 1 , 16); RotateReset();

        sleep(200);

        //Incepem sa le luam in ordine pana ajungem la unul negru
        StrafeUntilColor(0.15, 0, ColorSide, 9 , 16); RotateReset();
        StrafeWithEncoders(180, 0.2, 50); RotateReset();

        sleep(200);

        //Dam bratul in jos
        ServoBrat.setPosition(0.1);

        //Tragem ala in building zone
        StrafeWithEncoders(90, 0.6, 1500); RotateReset();
        StrafeUntilColor(0.5, 0, ColorUnderneath, 2 , 4); RotateReset();

        //Ridicam bratul
        RotateRelative(90);
        sleep(300);
        ServoBrat.setPosition(1);
        sleep(300);


        /*//Ne dam mai in drepta
        StrafeWithEncoders(-90, 0.6, 600); RotateReset();

        //Mergem in fata si rotim sa fim cu senzorul inspre caramizi
        StrafeWithEncoders(0, 0.5, 500); RotateReset();
        RotateRelative(90);

        //Mergem pana gasim ceva de culoarea galbena
        StrafeUntilColor(0.2, -90, ColorSide, 8); RotateReset();

        *//*sleep(500);

        //Mergem in fata pana trecem de primul stone
        StrafeWithAngle(0, 0, 0.3);
        while(opModeIsActive() && CheckHasLightInput(ColorSide)){
            idle();
        }
        StopWheels(); RotateReset();*//*

        sleep(500);

        //Incepem sa le luam in ordine pana ajungem la unul negru
        StrafeUntilColor(0.15, -180, ColorSide, 11); RotateReset();
        StrafeWithEncoders(180, 0.2, 75); RotateReset();

        sleep(500);

        //Dam bratul in jos
        ServoBrat.setPosition(0.1);

        //Tragem ala in building zone
        StrafeWithEncoders(90, 0.6, 1000); RotateReset();
        StrafeUntilColor(0.5, 0, ColorUnderneath, 3); RotateReset();
        sleep(300);
        //Ridicam bratul
        ServoBrat.setPosition(1);*/



    }
}
