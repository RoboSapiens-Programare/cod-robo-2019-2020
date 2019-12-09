package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test Servo")
public final class TestareServos extends LinearOpMode {
    private Servo servo1 = null;
    private Servo servo2 = null;
    private Servo servo3 = null;
    private CRServo crservo4 = null;
    private CRServo crservo5 = null;
    private CRServo crservo6 = null;

    boolean bForward = true;

    @Override
    public void runOpMode() throws InterruptedException {
        servo1 = hardwareMap.servo.get("servo1");
        servo2 = hardwareMap.servo.get("servo2");
        servo3 = hardwareMap.servo.get("servo3");
        servo1.setPosition(0.7);
        servo2.setPosition(0.7);
        servo3.setPosition(0);

        crservo4 = hardwareMap.crservo.get("crservo4");
        crservo5 = hardwareMap.crservo.get("crservo5");
        crservo6 = hardwareMap.crservo.get("crservo6");
        crservo4.setPower(0);
        crservo5.setPower(0);
        crservo6.setPower(0);

        waitForStart();

        while(opModeIsActive()){
            /*if(gamepad1.left_bumper){
                bForward = false;
            }
            else if(gamepad1.right_bumper){
                bForward = true;
            }*/

            if(gamepad1.dpad_up){
                servo1.setPosition(0.2);
                servo2.setPosition(0.2);
            }
            /*if(gamepad1.dpad_left){
                servo2.setPosition(bForward? 1 : 0);
            }
            if(gamepad1.dpad_right){
                servo3.setPosition(bForward? 1 : 0);
            }*/
            if(gamepad1.dpad_down){
                servo1.setPosition(0.8);
                servo2.setPosition(0.8);
            }


            if(gamepad1.a){
                crservo4.setPower(bForward? 1 : -1);
            }
            if(gamepad1.b){
                crservo5.setPower(bForward? 1 : -1);
            }
            if(gamepad1.y){
                crservo6.setPower(bForward? 1 : -1);
            }

            if(gamepad1.x){
                crservo4.setPower(0);
                crservo5.setPower(0);
                crservo6.setPower(0);
            }

            telemetry.addData("Forward", bForward);
            telemetry.update();

        }
    }
}
