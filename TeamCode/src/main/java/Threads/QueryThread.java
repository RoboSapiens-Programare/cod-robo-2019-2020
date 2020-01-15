package Threads;

import android.annotation.SuppressLint;
import android.os.HandlerThread;
import android.os.Message;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;

import android.os.Handler;

import java.lang.ref.WeakReference;
import java.util.logging.LogRecord;

public class QueryThread extends Thread {
    private static final String TAG = "QueryThread";

    ModernRoboticsI2cGyro gyro;
    private BrainThread parentBrainThread;

    //Handler handler;

    public QueryThread(ModernRoboticsI2cGyro gyro, BrainThread brainThread) {
        super("HandlerThread");
        this.gyro = gyro;
        parentBrainThread = brainThread;
    }

    @Override
    public void run() {
        super.run();
        while (this.isAlive() && parentBrainThread.isAlive()) {
            Message msg = Message.obtain(parentBrainThread.getHandler());
            msg.arg2 = gyro.getHeading();
            msg.what = 2;
            msg.sendToTarget();
        }

    }

    //    @SuppressLint("HandlerLeak")
//    @Override
//    protected void onLooperPrepared() {
//        handler = new Handler() {
//            @Override
//            public void handleMessage(Message msg) {
//                while ()
//            }
//        };
//    }
//
//    public Handler getHandler() {
//        return handler;
//    }

    public double returnAngle(){
        return gyro.getHeading();
    }







}
