package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name = "Bleh")
public final class Bleh extends RobotHardware {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        while (opModeIsActive()){


            telemetry.addData("Color read", ColorSide.read8(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER));
            telemetry.addData("Color uread", ColorUnderneath.read8(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER));
            telemetry.update();
        }
    }
}
