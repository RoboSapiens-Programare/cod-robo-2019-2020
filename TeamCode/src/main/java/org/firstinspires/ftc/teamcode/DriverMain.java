package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotHardware;

@TeleOp(name = "Driver Main")
public final class DriverMain extends RobotHardware {
    static final double DEADZONE = 0.1;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("EncoderL citeste: ", EncoderL.getDigitalInputStateByte());
            telemetry.addData("EncoderR citeste: ", EncoderR.getDigitalIOControlByte());
            telemetry.addData("EncoderUP citeste: ", EncoderUP.getDigitalOutputStateByte());
            telemetry.update();
        }
    }
}
