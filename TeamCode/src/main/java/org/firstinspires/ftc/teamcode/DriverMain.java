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
            StrafeWithAngle(Math.abs(gamepad1.left_stick_y) > DEADZONE? -gamepad1.left_stick_y : 0,
                            Math.abs(gamepad1.left_stick_x) > DEADZONE? gamepad1.left_stick_x : 0,
                            Math.abs(gamepad1.right_stick_x) > DEADZONE? gamepad1.right_stick_x : 0,
                            0.9);

            if(gamepad1.a){
                ServoBrat.setPosition(0.1);
            }
            else if(gamepad1.b){
                ServoBrat.setPosition(1);
            }

            

        }
    }




}
