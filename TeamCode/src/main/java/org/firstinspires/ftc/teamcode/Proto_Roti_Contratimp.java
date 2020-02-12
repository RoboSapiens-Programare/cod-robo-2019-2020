package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Test Roti Contratimp")
public class Proto_Roti_Contratimp extends LinearOpMode {
    private DcMotor LeftRoata, RightRoata;

    @Override
    public void runOpMode() throws InterruptedException {
        LeftRoata = hardwareMap.dcMotor.get("LeftRoata");
        RightRoata = hardwareMap.dcMotor.get("RightRoata");

        LeftRoata.setPower(0);
        RightRoata.setPower(0);

        LeftRoata.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RightRoata.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        LeftRoata.setDirection(DcMotorSimple.Direction.FORWARD);
        RightRoata.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.a) {
                LeftRoata.setPower(0.8);
                RightRoata.setPower(0.8);
            }
            else if(gamepad1.b){
                LeftRoata.setPower(-0.8);
                RightRoata.setPower(-0.8);
            }
            else{
                LeftRoata.setPower(0);
                RightRoata.setPower(0);
            }
        }

    }
}
