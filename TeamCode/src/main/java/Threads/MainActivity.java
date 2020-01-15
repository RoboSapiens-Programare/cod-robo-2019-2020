package Threads;

import android.os.HandlerThread;
import android.os.Message;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.logging.Handler;

@Autonomous(name = "ThreadYe")
public class MainActivity extends LinearOpMode {

    ModernRoboticsI2cGyro realGyro = null;

    @Override
    public void runOpMode() throws InterruptedException {

        realGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        realGyro.calibrate();

        waitForStart();

        BrainThread brainThread = new BrainThread(realGyro);
        brainThread.start();

        while (opModeIsActive()) {
            telemetry.addData("Target", brainThread.targetAngle);
            telemetry.addData("Herroeur:", brainThread.errorAngle);
            telemetry.addData("Error direct", 60 - realGyro.getHeading());
            telemetry.update();

            Message mainMsg = Message.obtain(brainThread.getHandler());
            mainMsg.arg1 = 60;
            mainMsg.what =1;
            mainMsg.sendToTarget();


        }

        brainThread.quit();
    }
}
