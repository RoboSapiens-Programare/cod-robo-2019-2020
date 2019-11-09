package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public abstract class RobotHardware extends LinearOpMode {
    //Motors
    protected DcMotor MotorFR = null;
    protected DcMotor MotorFL = null;
    protected DcMotor MotorBR = null;
    protected DcMotor MotorBL = null;

    //Servos
    protected Servo ServoBrat = null;

    //Sensors
    protected ModernRoboticsI2cRangeSensor RangeL = null;
    protected ModernRoboticsI2cRangeSensor RangeR = null;

    public void initialize(){
        MotorFR = hardwareMap.dcMotor.get("MotorFR");
        MotorFL = hardwareMap.dcMotor.get("MotorFL");
        MotorBR = hardwareMap.dcMotor.get("MotorBR");
        MotorBL = hardwareMap.dcMotor.get("MotorBL");

        MotorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorBL.setDirection(DcMotorSimple.Direction.REVERSE);

        MotorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MotorFR.setPower(0);
        MotorFL.setPower(0);
        MotorBR.setPower(0);
        MotorBL.setPower(0);

        MotorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RangeL = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "RangeL");
        RangeL.setI2cAddress(I2cAddr.create8bit(0x2a));
        RangeR = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "RangeR");
        RangeR.setI2cAddress(I2cAddr.create8bit(0x2c));

        ServoBrat = hardwareMap.servo.get("ServoBrat");
        ServoBrat.setPosition(1);
        ServoBrat.setDirection(Servo.Direction.FORWARD);

    }

    protected void StopWheels(){
        SetWheelsPower(0, 0,0,0);
    }

    protected void SetWheelsPower(double speedFR, double speedFL, double speedBR, double speedBL){
        MotorFR.setPower(speedFR);
        MotorBR.setPower(speedBR);
        MotorFL.setPower(speedFL);
        MotorBL.setPower(speedBL);
    }

    protected void SetWheelsPowerTank(double speedL, double speedR){
        MotorFR.setPower(speedR);
        MotorBR.setPower(speedR);
        MotorFL.setPower(speedL);
        MotorBL.setPower(speedL);
    }

    protected void SetWheelsPowerMeccanum(double speedFRBL, double speedFLBR){
        MotorFR.setPower(speedFRBL);
        MotorBR.setPower(speedFLBR);
        MotorFL.setPower(speedFRBL);
        MotorBL.setPower(speedFLBR);
    }

    protected void StrafeWithAngle(double angle, double rotate, double speed) {
        //transform angle to vectors
        double drive = Math.cos(angle);
        double strafe = Math.sin(angle);

        double FLBRNormal = Math.signum(drive)*Math.pow(drive,2) + Math.signum(strafe)*Math.pow(strafe,2);
        double FRBLNormal = Math.signum(drive)*Math.pow(drive,2) - Math.signum(strafe)*Math.pow(strafe,2);

        double ScalingCoefficient = 1;

        speed = Range.clip(speed, 0, 0.9);
        if(Math.max(Math.abs(FLBRNormal), Math.abs(FRBLNormal)) > speed) {
            ScalingCoefficient = speed / Math.max(Math.abs(FLBRNormal), Math.abs(FRBLNormal));
        }

        double SpeedFLBR = FLBRNormal * ScalingCoefficient;
        double SpeedFRBL = FRBLNormal * ScalingCoefficient;

        MotorFL.setPower(Range.clip(SpeedFLBR + rotate, -speed, speed));
        MotorFR.setPower(Range.clip(SpeedFRBL - rotate, -speed, speed));
        MotorBL.setPower(Range.clip(SpeedFRBL + rotate, -speed, speed));
        MotorBR.setPower(Range.clip(SpeedFLBR - rotate, -speed, speed));
    }

    protected void StrafeWithAngle(double drive, double strafe, double rotate, double maxspeed) {
        double FLBRNormal = Math.signum(drive)*Math.pow(drive,2) + Math.signum(strafe)*Math.pow(strafe,2);
        double FRBLNormal = Math.signum(drive)*Math.pow(drive,2) - Math.signum(strafe)*Math.pow(strafe,2);

        double ScalingCoefficient = 1;

        maxspeed = Range.clip(maxspeed, 0, 0.9);
        if(Math.max(Math.abs(FLBRNormal), Math.abs(FRBLNormal)) > maxspeed) {
            ScalingCoefficient = maxspeed / Math.max(Math.abs(FLBRNormal), Math.abs(FRBLNormal));
        }

        double SpeedFLBR = FLBRNormal * ScalingCoefficient;
        double SpeedFRBL = FRBLNormal * ScalingCoefficient;

        MotorFL.setPower(Range.clip(SpeedFLBR + rotate, -maxspeed, maxspeed));
        MotorFR.setPower(Range.clip(SpeedFRBL - rotate, -maxspeed, maxspeed));
        MotorBL.setPower(Range.clip(SpeedFRBL + rotate, -maxspeed, maxspeed));
        MotorBR.setPower(Range.clip(SpeedFLBR - rotate, -maxspeed, maxspeed));
    }
}
