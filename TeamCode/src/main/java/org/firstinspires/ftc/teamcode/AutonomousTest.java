package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomie Test")
public final class AutonomousTest extends AutonomousMain {
    final double DEADZONE = 0.1;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.a){
                StrafeWithEncoders(0, 0.4, 200);
                //StrafeWithObstacle(20);
            }
            if(gamepad1.b){
                StrafeWithEncoders(0, 0.4, 300);
                //StrafeWithEncoders(180, 0.4, 50);
                //StrafeWithObstacle(10);
            }
            if (gamepad1.y) {
                StrafeWithEncoders(0, 0.4, 400);
                //StrafeWithEncoders(90, 0.4, 50);
            }
            if (gamepad1.x){
                StrafeWithEncoders(0, 0.4, 500);
                //StrafeWithEncoders(45, 0.4, 100);
            }

            if (gamepad1.dpad_up) {
                Rotate(90);
                sleep(1000);
                Rotate(90);
                sleep(1000);
                Rotate(90);
                sleep(1000);
                Rotate(90);
                sleep(1000);
                Rotate(-45);
                sleep(1000);
                Rotate(-90);
                sleep(1000);
                Rotate(-135);
                sleep(1000);
                Rotate(-235);
            } else if (gamepad1.dpad_down) {
                Rotate(- 90);
            }

            StrafeWithAngle(Math.abs(gamepad1.left_stick_y) > DEADZONE? -gamepad1.left_stick_y : 0,
                    Math.abs(gamepad1.left_stick_x) > DEADZONE? gamepad1.left_stick_x : 0,
                    Math.abs(gamepad1.right_stick_x) > DEADZONE? gamepad1.right_stick_x : 0,
                    0.9);
        }
    }
}
