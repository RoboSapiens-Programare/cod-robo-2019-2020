package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous(name = "Autonomie Sky Stone Red Mijloc")
public final class AutonomousSkyStoneRedMijloc extends AutonomousMain {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        AutonomousSkyStoneRed();

        StrafeWithEncoders(0, 0.4, 400); RotateReset();

    }
}
