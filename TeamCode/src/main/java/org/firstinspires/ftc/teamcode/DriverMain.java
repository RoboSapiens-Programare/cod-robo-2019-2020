package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotHardware;

@TeleOp(name = "Driver Main")
public final class DriverMain extends RobotHardware {
    static final double DEADZONE = 0.1;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        while(opModeIsActive()){
            if(Math.abs(gamepad1.left_stick_x) > DEADZONE || Math.abs(gamepad1.left_stick_y) > DEADZONE || Math.abs(gamepad1.right_stick_x) > DEADZONE ) {
                PowerWheelsMecanum(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, 0.9);
            }
            else{
                StopWheels();
            }

        }
    }

    protected void PowerWheelsMecanum ( double drive, double strafe, double rotate, double maxspeed) {
        double FLBRNormal = Math.signum(drive)*Math.pow(drive,2) + Math.signum(strafe)*Math.pow(strafe,2);
        double FRBLNormal = Math.signum(drive)*Math.pow(drive,2) - Math.signum(strafe)*Math.pow(strafe,2);

        double ScalingCoefficient = 1;

        maxspeed = Range.clip(maxspeed, 0, 0.9);
        if(Math.max(Math.abs(FLBRNormal), Math.abs(FRBLNormal)) > maxspeed) {
            if (Math.max(Math.abs(FLBRNormal), Math.abs(FRBLNormal)) > DEADZONE) {
                ScalingCoefficient = maxspeed / Math.max(Math.abs(FLBRNormal), Math.abs(FRBLNormal));
            } else {
                ScalingCoefficient = 0;
            }
        }

        double SpeedFLBR = FLBRNormal * ScalingCoefficient;
        double SpeedFRBL = FRBLNormal * ScalingCoefficient;

        MotorFL.setPower(Range.clip(SpeedFLBR + rotate, -maxspeed, maxspeed));
        MotorFR.setPower(Range.clip(SpeedFRBL - rotate, -maxspeed, maxspeed));
        MotorBL.setPower(Range.clip(SpeedFRBL + rotate, -maxspeed, maxspeed));
        MotorBR.setPower(Range.clip(SpeedFLBR - rotate, -maxspeed, maxspeed));
    }

    protected void StopWheels(){
        MotorFR.setPower(0);
        MotorFL.setPower(0);
        MotorBR.setPower(0);
        MotorBL.setPower(0);
    }


}
