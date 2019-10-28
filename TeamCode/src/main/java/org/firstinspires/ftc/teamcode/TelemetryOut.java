package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRRangeSensor;

@Autonomous(name = "Telemetry")
public final class TelemetryOut extends LinearOpMode {
    ModernRoboticsI2cRangeSensor range1 = null;
    ModernRoboticsI2cRangeSensor range2 = null;

    @Override
    public void runOpMode() throws InterruptedException {
        range1 = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range1");
        //range2 = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range2");

        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("Range1: ", range1.rawUltrasonic());
            //telemetry.addData("Range2: ", range2.rawUltrasonic());
            telemetry.update();

        }
    }
}
