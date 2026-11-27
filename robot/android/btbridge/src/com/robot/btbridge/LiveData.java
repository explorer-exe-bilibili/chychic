package com.robot.btbridge;

/** 环形缓冲：Service 写入，界面读取。只保留最近 N 个采样点。 */
public class LiveData {

    public static final int CAP = 400;   // 20Hz 下约 20 秒

    private static final float[] sVL = new float[CAP];
    private static final float[] sVR = new float[CAP];
    private static final float[] sCmd = new float[CAP];
    private static int sCount = 0;
    private static int sHead = 0;

    public static volatile Proto.State latest;
    public static volatile Proto.LiftState latestLift;
    public static volatile int bleFrames = 0;
    public static volatile int bleCrcErrors = 0;
    public static volatile long lastFrameAt = 0;

    public static synchronized void push(Proto.State s) {
        latest = s;
        lastFrameAt = System.currentTimeMillis();
        sVL[sHead] = s.vLeft;
        sVR[sHead] = s.vRight;
        sCmd[sHead] = s.vCmd;
        sHead = (sHead + 1) % CAP;
        if (sCount < CAP) sCount++;
    }

    public static synchronized int size() { return sCount; }

    /** 按时间顺序拷贝一份，index 0 最旧。 */
    public static synchronized void snapshot(float[] vl, float[] vr, float[] cmd) {
        for (int i = 0; i < sCount; i++) {
            int idx = (sHead - sCount + i + CAP * 2) % CAP;
            vl[i] = sVL[idx];
            vr[i] = sVR[idx];
            cmd[i] = sCmd[idx];
        }
    }

    public static boolean online() {
        return lastFrameAt > 0 && System.currentTimeMillis() - lastFrameAt < 1000;
    }

    public static synchronized void reset() {
        sCount = 0;
        sHead = 0;
        latest = null;
    }
}
