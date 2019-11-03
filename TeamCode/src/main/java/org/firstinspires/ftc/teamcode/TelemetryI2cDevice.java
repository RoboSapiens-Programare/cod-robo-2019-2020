package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRRangeSensor;

@Autonomous(name = "I2c Telemetry")
public final class TelemetryI2cDevice extends LinearOpMode {
    I2cDevice range1 = null;
    I2cDevice range2 = null;
    I2cDeviceSynch range1Reader;
    I2cDeviceSynch range2Reader;

    @Override
    public void runOpMode() throws InterruptedException {
        range1 = hardwareMap.i2cDevice.get("range1");
        range2 = hardwareMap.i2cDevice.get("range2");

        range1Reader = new I2cDeviceSynchImpl(range1, I2cAddr.create8bit(0x29), false);
        range2Reader = new I2cDeviceSynchImpl(range2, I2cAddr.create8bit(0x2b), false);

        range1Reader.engage();
        range2Reader.engage();

        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("Range1: ", range1Reader.read(0x04, 2)[0] & 0xFF);
            telemetry.addData("Range2: ", range2Reader.read(0x04, 2)[0] & 0xFF);
            telemetry.update();

        }
    }
}
