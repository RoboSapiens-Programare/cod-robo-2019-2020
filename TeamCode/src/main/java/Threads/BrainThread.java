package Threads;

import android.annotation.SuppressLint;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.Message;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;

public class BrainThread extends HandlerThread {
    private static final String TAG = "BrainThread";

    //Handler mainHandler;

    ModernRoboticsI2cGyro brainGyro;

    QueryThread queryThread = new QueryThread(brainGyro);

    public BrainThread(ModernRoboticsI2cGyro brainGyro) {
        super("BrainThread");
        this.brainGyro = brainGyro;
    }

//    public static Handler getHandler() {
//        return mainHandler;
//    }
//
//
//    @SuppressLint("HandlerLeak")
//    @Override
//    protected void onLooperPrepared() {
//        mainHandler = new Handler() {
//            @Override
//            public void handleMessage(Message msg) {
////                switch (msg.what) {
////                    case EXAMPLE_TASK:
////                        Log.d(TAG, "Example Task, arg1: " + msg.arg1 + ", obj: " + msg.obj);
////                        for (int i = 0; i < 4; i++) {
////                            Log.d(TAG, "handleMessage: " + i);
////                            SystemClock.sleep(1000);
////                        }
////                        break;
////                }
//            }
//        };
//    }
}
