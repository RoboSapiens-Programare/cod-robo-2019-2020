package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Driver Main")
public final class DriverMain extends RobotHardware {
    static final double DEADZONE = 0.1;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        while(opModeIsActive()){
            if(Math.abs(gamepad1.left_stick_x) > DEADZONE || Math.abs(gamepad1.left_stick_y) > DEADZONE || Math.abs(gamepad1.right_stick_x) > DEADZONE ) {
                StrafeWithAngle(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, 0.9);
            }
            else{
                StopWheels();
            }

        }
    }




}
