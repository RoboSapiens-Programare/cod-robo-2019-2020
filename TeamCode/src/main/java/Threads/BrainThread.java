package Threads;

import android.os.Handler;
import android.os.HandlerThread;

public class BrainThread extends HandlerThread {
    private static final String TAG = "BrainThread";

    Handler mainHandler;

    //QueryThread queryThread = new QueryThread();

    public BrainThread() {
        super("BrainThread");
    }

}
