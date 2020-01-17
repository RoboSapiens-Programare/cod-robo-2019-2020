package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;

public abstract class RobotHardware extends LinearOpMode {
    protected DeviceInterfaceModule EncoderL = null;
    protected DeviceInterfaceModule EncoderR = null;
    protected DeviceInterfaceModule EncoderUP = null;

    public void initialize(){

        DigitalChannel L01;
        DigitalChannel L02;
        DigitalChannel R01;
        DigitalChannel R02;
        DigitalChannel UP01;
        DigitalChannel UP02;

        EncoderL = hardwareMap.get(DeviceInterfaceModule.class, "EncoderL");
        EncoderR = hardwareMap.get(DeviceInterfaceModule.class, "EncoderR");
        EncoderUP = hardwareMap.get(DeviceInterfaceModule.class, "EncoderUP");

        L01  = hardwareMap.get(DigitalChannel.class, "0");
        L02 = hardwareMap.get(DigitalChannel.class, "1");
        R01  = hardwareMap.get(DigitalChannel.class, "R01");
        R02 = hardwareMap.get(DigitalChannel.class, "R02");
        UP01  = hardwareMap.get(DigitalChannel.class, "UP01");
        UP02 = hardwareMap.get(DigitalChannel.class, "UP02");

        L01.setMode(DigitalChannel.Mode.OUTPUT);
        L02.setMode(DigitalChannel.Mode.OUTPUT);
        R01.setMode(DigitalChannel.Mode.OUTPUT);
        R02.setMode(DigitalChannel.Mode.OUTPUT);
        UP01.setMode(DigitalChannel.Mode.OUTPUT);
        UP02.setMode(DigitalChannel.Mode.OUTPUT);

    }

}
