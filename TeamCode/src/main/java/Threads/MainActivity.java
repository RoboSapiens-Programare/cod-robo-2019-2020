package Threads;

import android.os.HandlerThread;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.logging.Handler;

public class MainActivity extends LinearOpMode {

    ModernRoboticsI2cGyro realGyro = null;

    @Override
    public void runOpMode() throws InterruptedException {

        realGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");

        waitForStart();
        while (opModeIsActive()) {
            BrainThread brainThread = new BrainThread(realGyro);

        }

    }
}
