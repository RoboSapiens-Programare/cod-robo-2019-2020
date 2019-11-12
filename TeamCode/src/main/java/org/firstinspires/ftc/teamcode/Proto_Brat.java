package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Prototip Servo Brat")
public final class Proto_Brat extends LinearOpMode {
    //Servo ServoBrat = null;
    DcMotor motor = null;
    double DEADZONE = 0.1;

    @Override
    public void runOpMode() throws InterruptedException {
        //ServoBrat = hardwareMap.servo.get("ServoL");
        motor = hardwareMap.dcMotor.get("motor");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();

        while(opModeIsActive()){
            if(Math.abs(gamepad1.left_stick_y) > DEADZONE){
                //ServoBrat.setPosition(0);
                motor.setPower(Range.clip(gamepad1.left_stick_y, -0.9, 0.9));
            } else motor.setPower(0);
        }
    }
}
