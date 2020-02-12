package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Test Sasiu")
public class TestSasiu extends LinearOpMode {
    private DcMotor[] motors = new DcMotor[4];
    private String[] motorNames = {"MotorFL", "MotorFR", "MotorBL", "MotorBR"};
    BNO055IMU imu;
    private DcMotor LeftEncoder, RightEncoder, MiddleEncoder;

    public static double FLBRResult, FRBLResult;
    public static final double DEADZONE = 0.1;
    public enum MODE {
        DRIVE,
        INDIV,
        INDIV_CONST,
        TELEMETRY
    }
    MODE activeMode = MODE.DRIVE;
    double speedCursor = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        for(int i = 0; i < motors.length; i++){
            motors[i] = hardwareMap.dcMotor.get(motorNames[i]);
            motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motors[i].setPower(0);
        }

        motors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[1].setDirection(DcMotorSimple.Direction.FORWARD);
        motors[2].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[3].setDirection(DcMotorSimple.Direction.FORWARD);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        LeftEncoder = hardwareMap.dcMotor.get("LeftEncoder");
        RightEncoder = hardwareMap.dcMotor.get("RightEncoder");
        MiddleEncoder = hardwareMap.dcMotor.get("MiddleEncoder");


        waitForStart();

        while (opModeIsActive()){

            if(gamepad2.a){
                activeMode = MODE.DRIVE;
            }else if(gamepad2.b){
                activeMode = MODE.INDIV;
            }else if(gamepad2.x){
                activeMode = MODE.TELEMETRY;
            }else if(gamepad2.y){
                activeMode = MODE.INDIV_CONST;
            }

            switch (activeMode){
                case DRIVE:
                    StrafeWithAngle(Math.abs(gamepad1.left_stick_y) > DEADZONE? -gamepad1.left_stick_y : 0,
                            Math.abs(gamepad1.left_stick_x) > DEADZONE? gamepad1.left_stick_x : 0,
                            Math.abs(gamepad1.right_stick_x) > DEADZONE? gamepad1.right_stick_x : 0,
                            0.9);
                    break;

                case INDIV:
                    if(gamepad1.x){
                        motors[0].setPower(gamepad1.left_stick_y);
                    }
                    if(gamepad1.y){
                        motors[1].setPower(gamepad1.left_stick_y);
                    }
                    if(gamepad1.a){
                        motors[2].setPower(gamepad1.left_stick_y);
                    }
                    if(gamepad1.b){
                        motors[3].setPower(gamepad1.left_stick_y);
                    }

                    telemetry.addData("Y", gamepad1.left_stick_y);
                    telemetry.update();

                    break;

                case INDIV_CONST:
                    if(Math.abs(gamepad1.left_stick_y) > 0.1){
                        speedCursor += gamepad1.left_stick_y/100;
                        sleep(35);
                    }

                    if(gamepad1.x){
                        motors[0].setPower(speedCursor);
                    }
                    if(gamepad1.y){
                        motors[1].setPower(speedCursor);
                    }
                    if(gamepad1.a){
                        motors[2].setPower(speedCursor);
                    }
                    if(gamepad1.b){
                        motors[3].setPower(speedCursor);
                    }
                    if(gamepad1.right_bumper){
                        for(DcMotor motor : motors){
                            motor.setPower(0);
                        }
                    }

                    telemetry.addData("speedCursor", speedCursor);
                    telemetry.update();

                    break;

                case TELEMETRY:
                    StrafeWithAngle(Math.abs(gamepad1.left_stick_y) > DEADZONE? -gamepad1.left_stick_y : 0,
                            Math.abs(gamepad1.left_stick_x) > DEADZONE? gamepad1.left_stick_x : 0,
                            Math.abs(gamepad1.right_stick_x) > DEADZONE? gamepad1.right_stick_x : 0,
                            0.9);

                    telemetry.addData("Heading", imu.getAngularOrientation().firstAngle);
                    telemetry.addData("Left Encoder", LeftEncoder.getCurrentPosition());
                    telemetry.addData("Right Encoder", RightEncoder.getCurrentPosition());
                    telemetry.addData("Middle Encoder", MiddleEncoder.getCurrentPosition());
//                    telemetry.addData("MotorFL pow", motors[0].getPower());
//                    telemetry.addData("MotorFR pow", motors[1].getPower());
//                    telemetry.addData("MotorBL pow", motors[2].getPower());
//                    telemetry.addData("MotorBR pow", motors[3].getPower());
//                    telemetry.addLine();
//                    telemetry.addData("MotorFL enc", motors[0].getCurrentPosition());
//                    telemetry.addData("MotorFR enc", motors[1].getCurrentPosition());
//                    telemetry.addData("MotorBL enc", motors[2].getCurrentPosition());
//                    telemetry.addData("MotorBR enc", motors[3].getCurrentPosition());
                    telemetry.update();
                    break;
            }
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

    protected void CalculateMecanumResult(double y, double x){
        FRBLResult = Math.signum(y)*Math.pow(y,2) - Math.signum(x)*Math.pow(x,2);
        FLBRResult = Math.signum(y)*Math.pow(y,2) + Math.signum(x)*Math.pow(x,2);
    }
}
