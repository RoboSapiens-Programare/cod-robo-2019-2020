package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Driver Main")
public final class DriverMain extends RobotHardware {
    static final double DEADZONE = 0.1;
    double YAxisPosition = 0.4;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        while (opModeIsActive()) {
            Gamepad1();
            Gamepad2();

            telemetry.addData("YAxisParam", YAxisPosition);
            telemetry.update();

        }
    }

    private void Gamepad1(){
        StrafeWithAngle(Math.abs(gamepad1.left_stick_y) > DEADZONE? -gamepad1.left_stick_y : 0,
                Math.abs(gamepad1.left_stick_x) > DEADZONE? gamepad1.left_stick_x : 0,
                Math.abs(gamepad1.right_stick_x) > DEADZONE? gamepad1.right_stick_x : 0,
                0.9);
    }

    private void Gamepad2(){
        //Servos
        if(gamepad2.a){
            ServoBrat.setPosition(0.1);
        }
        else if(gamepad2.b){
            ServoBrat.setPosition(1);
        }

        if(gamepad2.dpad_left){
            ServoZAxis.setPosition(1);
        }
        else if(gamepad2.dpad_right){
            ServoZAxis.setPosition(0.4);
        }

        if (Math.abs(gamepad2.left_stick_y) > 0.1){
            ServoYAxis.setPower(gamepad2.left_stick_y);
        }
        else {
            ServoYAxis.setPower(0);
        }


        if(gamepad2.left_bumper){
            ServoClampLeft.setPosition(0);
            ServoClampRight.setPosition(0);
        }
        else if(gamepad2.right_bumper){
            ServoClampLeft.setPosition(0.4);
            ServoClampRight.setPosition(0.4);
        }

        //Motors
        if(gamepad2.left_trigger > DEADZONE){
            MotorBratColectare.setPower(Range.clip(gamepad2.left_trigger / 2, 0, 0.5));
        } else if(gamepad2.right_trigger > DEADZONE){
            MotorBratColectare.setPower(Range.clip(-gamepad2.right_trigger / 2, -0.5, 0));
        } else
            MotorBratColectare.setPower(0);
    }




}
