package Threads;

import android.os.HandlerThread;
import android.os.Message;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;

import java.util.logging.Handler;

public class QueryThread extends HandlerThread {
    private static final String TAG = "QueryThread";

    ModernRoboticsI2cGyro gyro;

    Handler handler;

    public QueryThread( ModernRoboticsI2cGyro gyro) {
        super("HandlerThread");
        this.gyro = gyro;
    }

    public Handler getHandler() {
        return handler;
    }

    public double returnAngle(){
        return gyro.getHeading();
    }

    Message msg = Message.obtain(BrainThread.getHandler());





}
