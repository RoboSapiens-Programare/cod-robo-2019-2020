package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomie Sky Stone")
public final class AutonomousSkyStone extends AutonomousMain {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        //Ne dam mai in drepta
        StrafeWithEncoders(-90, 0.6, 1000);

        //Mergem in fata si rotim sa fim cu senzorul inspre caramizi
        StrafeWithEncoders(0, 0.3, 600);
        RotateRelative(90);

        //Mergem pana gasim ceva de culoarea galbena
        StrafeUntilColor(0.5, -90, ColorSide, 8);

        //Mergem in fata pana trecem de primul stone
        StrafeWithAngle(0, 0, 0.3);
        while(opModeIsActive() && CheckHasLightInput(ColorSide)){
            idle();
        }
        StopWheels();

        //Incepem sa le luam in ordine pana ajungem la unul negru
        StrafeUntilColor(0.3, -180, ColorSide, 10);

        //Dam bratul in jos
        ServoBrat.setPosition(0.1);

        //Tragem ala in building zone
        StrafeWithEncoders(90, 0.6, 1000);
        RotateAbsolute(0);
        StrafeToObstacle(100);

        //Ridicam bratul
        ServoBrat.setPosition(1);

    }
}
