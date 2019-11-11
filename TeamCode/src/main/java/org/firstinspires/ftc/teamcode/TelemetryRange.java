package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceReader;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Range Telemetry")
public class TelemetryRange extends LinearOpMode {
    ModernRoboticsI2cRangeSensor range1;
    ModernRoboticsI2cRangeSensor range2;

    @Override
    public void runOpMode() throws InterruptedException {
        // get a reference to our compass
        range1 = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "RangeL");
        range2 = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "RangeR");

        range1.setI2cAddress(I2cAddr.create8bit(0x2c));
        range2.setI2cAddress(I2cAddr.create8bit(0x2e5));

        //ModernRoboticsI2cRangeSensor.Register reg = new ModernRoboticsI2cRangeSensor.Register();

        // wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Range Address ", range1.getI2cAddress().get8Bit());
            telemetry.addData("raw ultrasonic", range1.rawUltrasonic());
            telemetry.addData("raw optical", range1.rawOptical());
            telemetry.addData("cm optical", "%.2f cm", range1.cmOptical());
            telemetry.addData("cm", "%.2f cm", range1.getDistance(DistanceUnit.CM));

            telemetry.addLine();

            telemetry.addData("Range Address ", range2.getI2cAddress().get8Bit());
            telemetry.addData("raw ultrasonic", range2.rawUltrasonic());
            telemetry.addData("raw optical", range2.rawOptical());
            telemetry.addData("cm optical", "%.2f cm", range2.cmOptical());
            telemetry.addData("cm", "%.2f cm", range2.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}
