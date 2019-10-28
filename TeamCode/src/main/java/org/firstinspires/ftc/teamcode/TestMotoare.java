package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;



@TeleOp(name = "TestMotoare")


public class TestMotoare extends RobotHardware {



    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        waitForStart();

        while(opModeIsActive()) {
            if (gamepad1.x) {
                MotorBL.setPower(0.3);
            }
            if (gamepad1.y) {
                MotorFL.setPower(0.3);
            }
            if (gamepad1.a) {
                MotorBR.setPower(0.3);
            }
            if (gamepad1.b) {
                MotorFR.setPower(0.3);
            }
        }
    }
}
