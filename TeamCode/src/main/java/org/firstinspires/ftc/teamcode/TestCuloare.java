package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.AdafruitI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class TestCuloare extends LinearOpMode {

    ColorSensor Color = null;

    @Override
    public void runOpMode() throws InterruptedException {
        Color = hardwareMap.colorSensor.get("color");
        Color.enableLed(true);

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Nr culoare: ", Color.argb());
            telemetry.update();
        }


    }
}
