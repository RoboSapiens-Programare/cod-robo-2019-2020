package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name = "Bleh")
public final class Bleh extends RobotHardware {
    static final double DEADZONE = 0.1;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        while (opModeIsActive()){
            StrafeWithAngle(Math.abs(gamepad1.left_stick_y) > DEADZONE? -gamepad1.left_stick_y : 0,
                    Math.abs(gamepad1.left_stick_x) > DEADZONE? gamepad1.left_stick_x : 0,
                    Math.abs(gamepad1.right_stick_x) > DEADZONE? gamepad1.right_stick_x : 0,
                    0.7);

            telemetry.addData("Color read", ColorSide.read8(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER));
            telemetry.addData("Color uread", ColorUnderneath.read8(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER));
            telemetry.update();
        }
    }
}
