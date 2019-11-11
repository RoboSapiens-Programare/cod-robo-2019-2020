package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.AdafruitI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
 @Autonomous (name = "TestCuloare")

public class  TestCuloare extends LinearOpMode {

     ModernRoboticsI2cColorSensor Color;

    @Override
    public void runOpMode() throws InterruptedException {
        Color = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "color");
        Color.enableLed(false);

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("argb: ", Color.argb());
            telemetry.addData("alpha: ", Color.alpha());
            telemetry.addData("red: ", Color.red());
            telemetry.addData("green: ", Color.green());
            telemetry.addData("blue: ", Color.blue());
            telemetry.addData("numar: ", Color.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER));
            telemetry.update();
        }


    }
}
