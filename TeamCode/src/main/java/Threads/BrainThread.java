package Threads;

import android.annotation.SuppressLint;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.Message;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;

public class BrainThread extends HandlerThread {
    private static final String TAG = "BrainThread";

    public int targetAngle, errorAngle;

    private Handler mainHandler;

//    ModernRoboticsI2cGyro brainGyro;

    QueryThread queryThread;

    public BrainThread(ModernRoboticsI2cGyro brainGyro) {
        super("BrainThread");
//        this.brainGyro = brainGyro;
        queryThread = new QueryThread(brainGyro, this);
    }

    public Handler getHandler() {
        return mainHandler;
    }

    @SuppressLint("HandlerLeak")
    @Override
    protected void onLooperPrepared() {
        mainHandler = new Handler() {
            @Override
            public void handleMessage(Message msg) {
                switch (msg.what) {
                    case 1:
                        targetAngle = msg.arg1;

                        break;

                    case 2:
                        errorAngle = targetAngle - msg.arg2;

                        break;
                }
            }
        };

        queryThread.start();

    }




}
