package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public abstract class RobotHardware extends LinearOpMode {
    protected DcMotor EncoderL = null;
    protected DcMotor EncoderR = null;
    protected DcMotor EncoderUP = null;

    public void initialize(){
        EncoderL = hardwareMap.dcMotor.get("EncoderL");
        EncoderR = hardwareMap.dcMotor.get("EncoderR");
        EncoderUP = hardwareMap.dcMotor.get("EncoderUP");

        EncoderL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        EncoderR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        EncoderUP.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        EncoderL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        EncoderR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        EncoderUP.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        EncoderL.setPower(0);
        EncoderR.setPower(0);
        EncoderUP.setPower(0);

        EncoderL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        EncoderR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        EncoderUP.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        EncoderL.setDirection(DcMotorSimple.Direction.FORWARD);
        EncoderUP.setDirection(DcMotorSimple.Direction.FORWARD);
        EncoderR.setDirection(DcMotorSimple.Direction.REVERSE);
    }

}
