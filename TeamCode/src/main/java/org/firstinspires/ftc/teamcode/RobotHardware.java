package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
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

    protected DcMotor MotorVerticalLeft;
    protected DcMotor MotorVerticalRight;
    protected DcMotor MotorHorizontal;

    //Servos
    protected Servo ServoBrat = null;
    protected Servo ServoZAxis = null;
    protected Servo ServoClampLeft = null;
    protected Servo ServoClampRight = null;
    protected Servo ServoTavaRight = null;
    protected Servo ServoTavaLeft = null;

    //Sensors
    protected ModernRoboticsI2cRangeSensor RangeL = null;
    protected ModernRoboticsI2cRangeSensor RangeR = null;
    protected ModernRoboticsI2cColorSensor ColorSide = null;
    protected ModernRoboticsI2cColorSensor ColorUnderneath = null;

    //Gyro
    protected BNO055IMU Gyro = null;

    //Global Variables
    Orientation lastAngles = new Orientation();
    double globalAngle;
    protected double globalTargetAngle = 0;
    static double FRBLResult;
    static double FLBRResult;
    double valAlphaMIN = 5;


    public void initialize(){
        //Motoare roti
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

        //Motoare Odometrie
        MotorVerticalLeft = hardwareMap.dcMotor.get("a");
        MotorVerticalRight = hardwareMap.dcMotor.get("b");
        MotorHorizontal = hardwareMap.dcMotor.get("c");

        MotorVerticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorVerticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorHorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        MotorVerticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorVerticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorHorizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        MotorVerticalLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorVerticalRight.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorHorizontal.setDirection(DcMotorSimple.Direction.REVERSE);

        //Brat Colectare
        MotorBratColectare = hardwareMap.dcMotor.get("MotorBrat");
        MotorBratColectare.setPower(0);
        MotorBratColectare.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorBratColectare.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorBratColectare.setDirection(DcMotorSimple.Direction.FORWARD);


        ServoZAxis = hardwareMap.servo.get("ServoZAxis");
        ServoZAxis.setPosition(0.5);

        ServoClampLeft = hardwareMap.servo.get("ServoClampLeft");
        ServoClampLeft.setPosition(0.4);

        ServoClampRight = hardwareMap.servo.get("ServoClampRight");
        ServoClampRight.setPosition(0.4);

        ServoTavaRight = hardwareMap.servo.get("ServoTavaRight");
        ServoTavaRight.setPosition(0.1);

        ServoTavaLeft = hardwareMap.servo.get("ServoTavaLeft");
        ServoTavaLeft.setPosition(1);


        RangeL = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "RangeL");
        RangeL.setI2cAddress(I2cAddr.create8bit(0x2c));
        RangeL.enableLed(true);
        RangeR = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "RangeR");
        RangeR.setI2cAddress(I2cAddr.create8bit(0x2e));
        RangeR.enableLed(true);

        ColorSide = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "ColorSide");
        ColorSide.enableLed(true);
        ColorUnderneath = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "ColorUnderneath");
        ColorUnderneath.enableLed(true);

        ServoBrat = hardwareMap.servo.get("ServoBrat");
        ServoBrat.setPosition(1);

        Gyro = hardwareMap.get(BNO055IMU.class, "imu");


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

    protected void SetWheelsPowerMecanum(double speedFRBL, double speedFLBR){
        MotorFR.setPower(speedFRBL);
        MotorBR.setPower(speedFLBR);
        MotorFL.setPower(speedFRBL);
        MotorBL.setPower(speedFLBR);
    }

    protected void StrafeWithAngle(double angle, double rotate, double speed) {

        angle += 90;

        //transform angle to vectors
        double drive = Math.sin(Math.toRadians(angle));
        double strafe = Math.cos(Math.toRadians(angle));

        CalculateMecanumResult(drive, strafe);

        double FLBRNormal = FLBRResult;
        double FRBLNormal = FRBLResult;

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

        globalAngle += angles.firstAngle - lastAngles.firstAngle;
        globalAngle %= 360;

        if (globalAngle < -180)
            globalAngle += 360;
        else if (globalAngle > 180)
            globalAngle -= 360;

        lastAngles = angles;

        return globalAngle;
    }

    //Function that resets the global angle to 0
    protected void ResetAngle() {
        lastAngles = Gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
        globalTargetAngle = 0;
    }

    //Function that calculates mecanum vectors based on XY coordinates
    protected void CalculateMecanumResult(double y, double x){
        FRBLResult = Math.signum(y)*Math.pow(y,2) - Math.signum(x)*Math.pow(x,2);
        FLBRResult = Math.signum(y)*Math.pow(y,2) + Math.signum(x)*Math.pow(x,2);
    }

    protected void ResetAllEncoders() {
        MotorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    protected void RunWithAllEncoders() {
        MotorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

}
