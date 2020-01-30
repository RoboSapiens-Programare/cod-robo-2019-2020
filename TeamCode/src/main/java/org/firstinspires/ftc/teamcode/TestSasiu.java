package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class TestSasiu extends LinearOpMode {
    DcMotor[] motors = new DcMotor[4];
    String[] motorNames = {"MotorFL", "MotorFR", "MotorBL", "MotorBR"};

    @Override
    public void runOpMode() throws InterruptedException {
        for(int i = 0; i < motors.length; i++){
            motors[i] = hardwareMap.dcMotor.get(motorNames[i]);
            motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motors[i].setPower(0);
        }

        motors[2].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[0].setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();

        while (opModeIsActive()){
            
        }

    }
}
