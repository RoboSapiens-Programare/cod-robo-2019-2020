package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

public class TestSasiu extends LinearOpMode {
    DcMotor[] motors = new DcMotor[4];
    String[] motorNames = {"MotorFL", "MotorFR", "MotorBL", "MotorBR"};

    public static double FLBRResult, FRBLResult;

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

    protected void StrafeWithAngle(double drive, double strafe, double rotate, double maxspeed) {
        CalculateMecanumResult(drive, strafe);

        double FLBRNormal = FLBRResult;
        double FRBLNormal = FRBLResult;

        double ScalingCoefficient = 1;

        maxspeed = Range.clip(maxspeed, 0, 0.9);
        if(Math.max(Math.abs(FLBRNormal), Math.abs(FRBLNormal)) > maxspeed) {
            ScalingCoefficient = maxspeed / Math.max(Math.abs(FLBRNormal), Math.abs(FRBLNormal));
        }

        double SpeedFLBR = FLBRNormal * ScalingCoefficient;
        double SpeedFRBL = FRBLNormal * ScalingCoefficient;

        motors[0].setPower(Range.clip(SpeedFLBR + rotate, -maxspeed, maxspeed));
        motors[1].setPower(Range.clip(SpeedFRBL - rotate, -maxspeed, maxspeed));
        motors[2].setPower(Range.clip(SpeedFRBL + rotate, -maxspeed, maxspeed));
        motors[3].setPower(Range.clip(SpeedFLBR - rotate, -maxspeed, maxspeed));
    }
}
