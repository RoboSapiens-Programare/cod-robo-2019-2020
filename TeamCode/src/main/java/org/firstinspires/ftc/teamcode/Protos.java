package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp (name = "PrototipColectare")
public class Protos extends LinearOpMode {
    Servo ServoL = null;
    Servo ServoR = null;
    Servo ServoZAxis = null;
    Servo ServoYAxis = null;

    @Override
    public void runOpMode() {
        ServoL = hardwareMap.servo.get("ServoL");
        ServoR = hardwareMap.servo.get("ServoR");
        ServoZAxis = hardwareMap.servo.get("ServoZAxis");
        ServoYAxis = hardwareMap.servo.get("ServoYAxis");

        ServoL.setDirection(Servo.Direction.FORWARD);
        ServoR.setDirection(Servo.Direction.FORWARD);
        ServoZAxis.setDirection(Servo.Direction.FORWARD);
        ServoYAxis.setDirection(Servo.Direction.FORWARD);

        ServoYAxis.setPosition(0.5);
        ServoZAxis.setPosition(0.35);

        waitForStart();



        while (opModeIsActive()) {
            Gamepad1();
        }
    }

    protected void Gamepad1() {


        if (gamepad1.x){
            ServoL.setPosition(0);
            ServoR.setPosition(0);
        }
        else if (gamepad1.a){
            ServoL.setPosition(0.3);
            ServoR.setPosition(0.5);
        }
        else if (gamepad1.dpad_up){
            ServoZAxis.setPosition(0.5);
        }
        else if (gamepad1.dpad_down){
            ServoZAxis.setPosition(1);
        }
        else if (gamepad1.dpad_left){
            ServoYAxis.setPosition(0.35);
        }
        else if (gamepad1.dpad_right) {
            ServoYAxis.setPosition(1);
        }

    }
}
