package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp ( name = "Test sasiu turbo")
public class TestSasiuTurbo extends LinearOpMode {
    DcMotor[] motors = new DcMotor[4];
    String[] motorNames = {"MotorFL", "MotorFR", "MotorBL", "MotorBR"};

    @Override
    public void runOpMode() throws InterruptedException {
        for(int i = 0; i < motors.length; i++){
            motors[i] = hardwareMap.dcMotor.get(motorNames[i]);
            motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motors[i].setPower(0);
        }

        //motors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[1].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[2].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[3].setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            gamepad1();
            telemetry.update();
//            for ( int i = 0; i < 4; ++i ) {
//                motors[i].setPower(0.8);
//            }
            telemetry.addData("MotorFL: ",motors[0].getCurrentPosition());
            telemetry.addData("MotorFR: ",motors[1].getCurrentPosition());
            telemetry.addData("MotorBL: ",motors[2].getCurrentPosition());
            telemetry.addData("MotorBR: ",motors[3].getCurrentPosition());
        }


//        for ( int i = 0; i < 4; ++i ) {
//            motors[i].setPower(0);
//        }
    }

    public void gamepad1() {
        if (gamepad1.x){
            motors[0].setPower(0.8);
        }
        if (gamepad1.y){
            motors[1].setPower(0.8);
        }
        if (gamepad1.a){
            motors[2].setPower(0.8);
        }
        if (gamepad1.b){
            motors[3].setPower(0.8);
        }
    }
}
