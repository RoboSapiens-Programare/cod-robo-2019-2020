package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//@Disabled
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

            telemetry.addData("ColorSide read", ColorSide.read8(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER));
            telemetry.addData("ColorSide read GREEN", ColorSide.green());
            telemetry.addData("ColorSide read RED", ColorSide.red());
            telemetry.addData("ColorSide read BLUE", ColorSide.blue());

            telemetry.addData("ColorUnderneath read", ColorUnderneath.read8(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER));
            telemetry.addData("ColorUnderneath read GREEN", ColorUnderneath.green());
            telemetry.addData("ColorUnderneath read RED", ColorUnderneath.red());
            telemetry.addData("ColorUnderneath read BLUE", ColorUnderneath.blue());
            telemetry.addData("ColorUnderneath read normalized", ColorUnderneath.getNormalizedColors().toColor());

            telemetry.update();
        }
    }
}
