package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public abstract class RobotHardware extends LinearOpMode {
    //Motors
    protected DcMotor MotorFR = null;
    protected DcMotor MotorFL = null;
    protected DcMotor MotorBR = null;
    protected DcMotor MotorBL = null;
    protected DcMotor MotorBratColectare = null;

    //Servos
    protected Servo ServoBrat = null;
    protected CRServo ServoYAxis = null;
    protected Servo ServoZAxis = null;
    protected Servo ServoClampLeft = null;
    protected Servo ServoClampRight = null;

    //Sensors
    protected ModernRoboticsI2cRangeSensor RangeL = null;
    protected ModernRoboticsI2cRangeSensor RangeR = null;

    //Gyro
    protected BNO055IMU Gyro = null;

    //Constants
    Orientation lastAngles = new Orientation();
    double globalAngle;


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

        //Brat Colectare
        MotorBratColectare = hardwareMap.dcMotor.get("MotorBrat");
        MotorBratColectare.setPower(0);
        MotorBratColectare.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorBratColectare.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ServoYAxis = hardwareMap.crservo.get("ServoYAxis");
        ServoYAxis.setPower(0.0);


        ServoZAxis = hardwareMap.servo.get("ServoZAxis");
        ServoZAxis.setPosition(0);

        ServoClampLeft = hardwareMap.servo.get("ServoClampLeft");
        ServoClampLeft.setPosition(0);

        ServoClampRight = hardwareMap.servo.get("ServoClampRight");
        ServoClampRight.setPosition(0);


        RangeL = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "RangeL");
        RangeL.setI2cAddress(I2cAddr.create8bit(0x2c));
        RangeR = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "RangeR");
        RangeR.setI2cAddress(I2cAddr.create8bit(0x2e));

        ServoBrat = hardwareMap.servo.get("ServoBrat");
        ServoBrat.setPosition(1);
        ServoBrat.setDirection(Servo.Direction.FORWARD);

        Gyro = hardwareMap.get( BNO055IMU.class, "imu");


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

    protected void CalibrateGyro(){
        BNO055IMU.Parameters REVGyroParameters = new BNO055IMU.Parameters();

        REVGyroParameters.mode = BNO055IMU.SensorMode.IMU;
        REVGyroParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        REVGyroParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        REVGyroParameters.loggingEnabled = false;


        Gyro.initialize(REVGyroParameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !Gyro.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        ResetAngle();

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", Gyro.getCalibrationStatus().toString());
        telemetry.update();
    }

    //Function that adds the orientation of the REV integrated gyro to the globalAngle variable
    protected double GetAngle() {
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = Gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    //Function that resets the global angle to 0
    protected void ResetAngle() {
        lastAngles = Gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

}
