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
                StrafeWithObstacle(20);
            }
            if(gamepad1.b){
                StrafeWithObstacle(10);
            }

            StrafeWithAngle(Math.abs(gamepad1.left_stick_y) > DEADZONE? -gamepad1.left_stick_y : 0,
                    Math.abs(gamepad1.left_stick_x) > DEADZONE? gamepad1.left_stick_x : 0,
                    Math.abs(gamepad1.right_stick_x) > DEADZONE? gamepad1.right_stick_x : 0,
                    0.9);
        }
    }
}
