package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Prototip Servo Brat")
public final class Proto_Brat extends LinearOpMode {
    Servo ServoBrat = null;

    @Override
    public void runOpMode() throws InterruptedException {
        ServoBrat = hardwareMap.servo.get("ServoL");

        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.a){
                ServoBrat.setPosition(0);
            }
            else if(gamepad1.b){
                ServoBrat.setPosition(1);
            }
        }
    }
}
