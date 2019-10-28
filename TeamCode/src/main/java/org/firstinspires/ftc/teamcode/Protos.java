package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp (name = "PrototipColectare")
public class Protos extends LinearOpMode {
    Servo ServoL = null;
    Servo ServoR = null;

    @Override
    public void runOpMode() {
        ServoL = hardwareMap.servo.get("ServoL");
        ServoR = hardwareMap.servo.get("ServoR");

        ServoL.setDirection(Servo.Direction.FORWARD);
        ServoR.setDirection(Servo.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {
            Gamepad1();

        }
    }

    protected void Gamepad1() {

        if (gamepad1.x){
            ServoL.setPosition(0);
        }
        else if (gamepad1.y){
            ServoR.setPosition(0);
        }
        else if (gamepad1.a){
            ServoL.setPosition(1);
        }
        else if (gamepad1.b){
            ServoR.setPosition(1);
        }

    }
}
