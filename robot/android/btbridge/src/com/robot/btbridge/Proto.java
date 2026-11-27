package com.robot.btbridge;

import java.util.ArrayList;
import java.util.List;

/**
 * 协议解析：与上位机 protocol.py、固件 protocol.c 保持一致。
 * 只解析 App 画图需要的两类帧，其余原样透传给 TCP 客户端。
 */
public class Proto {

    public static final int FB_STATE = 0x80;
    public static final int FB_LIFT_STATE = 0x82;

    public static final int MODE_IDLE = 0;
    public static final int MODE_LINE_FOLLOW = 1;
    public static final int MODE_MANUAL = 2;
    public static final int MODE_ESTOP = 3;

    /** 底盘状态快照，字段含义见 robot/docs/protocol.md */
    public static class State {
        public int tickMs;
        public int mode;
        public int lfState;
        public int lineMask;
        public float lfError;
        public float vCmd;
        public float wCmd;
        public float vLeft;
        public float vRight;
        public int encLeft;
        public int encRight;
        public int status;

        public boolean watchdog() { return (status & 0x04) != 0; }
        public boolean estop()    { return (status & 0x08) != 0; }
    }

    /** 升降台状态快照 */
    public static class LiftState {
        public int tickMs;
        public float positionMm;
        public int targetMm;
        public int encoder;
        public int state;
        public int flags;
        public boolean referenced() { return (flags & 0x02) != 0; }
    }

    public static class Frame {
        public final int cmd;
        public final byte[] payload;
        public Frame(int cmd, byte[] payload) { this.cmd = cmd; this.payload = payload; }
    }

    /** 逐字节帧解析器。对垃圾数据免疫，失步后会重新找 0xAA 0x55。 */
    public static class Decoder {
        private final byte[] buf = new byte[512];
        private int len = 0;
        public int frames = 0;
        public int crcErrors = 0;

        public List<Frame> feed(byte[] data, int n) {
            List<Frame> out = new ArrayList<>();
            for (int i = 0; i < n; i++) {
                if (len >= buf.length) len = 0;
                buf[len++] = data[i];
                extract(out);
            }
            return out;
        }

        private void extract(List<Frame> out) {
            while (true) {
                int start = -1;
                for (int i = 0; i + 1 < len; i++) {
                    if ((buf[i] & 0xFF) == 0xAA && (buf[i + 1] & 0xFF) == 0x55) { start = i; break; }
                }
                if (start < 0) {
                    if (len > 1) { buf[0] = buf[len - 1]; len = 1; }
                    return;
                }
                if (start > 0) {
                    System.arraycopy(buf, start, buf, 0, len - start);
                    len -= start;
                }
                if (len < 5) return;

                int plen = buf[4] & 0xFF;
                if (plen > 64) { drop(1); continue; }
                int total = 7 + plen;
                if (len < total) return;

                int crc = crc16(buf, 2, 3 + plen);
                int rx = ((buf[5 + plen] & 0xFF) << 8) | (buf[6 + plen] & 0xFF);
                if (crc != rx) { crcErrors++; drop(1); continue; }

                byte[] payload = new byte[plen];
                System.arraycopy(buf, 5, payload, 0, plen);
                out.add(new Frame(buf[3] & 0xFF, payload));
                frames++;
                drop(total);
            }
        }

        private void drop(int n) {
            System.arraycopy(buf, n, buf, 0, len - n);
            len -= n;
        }
    }

    public static int crc16(byte[] d, int off, int n) {
        int crc = 0xFFFF;
        for (int i = 0; i < n; i++) {
            crc ^= (d[off + i] & 0xFF) << 8;
            for (int b = 0; b < 8; b++) {
                crc = ((crc & 0x8000) != 0) ? ((crc << 1) ^ 0x1021) : (crc << 1);
                crc &= 0xFFFF;
            }
        }
        return crc;
    }

    private static int u32(byte[] p, int o) {
        return ((p[o] & 0xFF) << 24) | ((p[o + 1] & 0xFF) << 16)
             | ((p[o + 2] & 0xFF) << 8) | (p[o + 3] & 0xFF);
    }

    private static float f32(byte[] p, int o) {
        return Float.intBitsToFloat(u32(p, o));
    }

    private static int i32(byte[] p, int o) { return u32(p, o); }

    /** 解析 FB_STATE（36 字节）。长度不符返回 null。 */
    public static State parseState(byte[] p) {
        if (p == null || p.length != 36) return null;
        State s = new State();
        s.tickMs = u32(p, 0);
        s.mode = p[4] & 0xFF;
        s.lfState = p[5] & 0xFF;
        s.lineMask = p[6] & 0xFF;
        s.lfError = f32(p, 7);
        s.vCmd = f32(p, 11);
        s.wCmd = f32(p, 15);
        s.vLeft = f32(p, 19);
        s.vRight = f32(p, 23);
        s.encLeft = i32(p, 27);
        s.encRight = i32(p, 31);
        s.status = p[35] & 0xFF;
        return s;
    }

    /** 解析 FB_LIFT_STATE（20 字节）。 */
    public static LiftState parseLift(byte[] p) {
        if (p == null || p.length != 20) return null;
        LiftState s = new LiftState();
        s.tickMs = u32(p, 0);
        s.positionMm = f32(p, 4);
        s.targetMm = ((p[8] & 0xFF) << 8) | (p[9] & 0xFF);
        s.encoder = i32(p, 10);
        s.state = p[14] & 0xFF;
        s.flags = p[16] & 0xFF;
        return s;
    }
}
