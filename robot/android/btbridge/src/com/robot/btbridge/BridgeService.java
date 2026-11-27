package com.robot.btbridge;

import android.app.Notification;
import android.app.NotificationChannel;
import android.app.NotificationManager;
import android.app.Service;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCallback;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattDescriptor;
import android.bluetooth.BluetoothGattService;
import android.bluetooth.BluetoothManager;
import android.bluetooth.le.BluetoothLeScanner;
import android.bluetooth.le.ScanCallback;
import android.bluetooth.le.ScanResult;
import android.content.Context;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.os.Build;
import android.os.IBinder;
import android.util.Log;

import java.io.InputStream;
import java.io.OutputStream;
import java.net.InetAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.List;
import java.util.UUID;

/**
 * 把 BLE 透传模块桥接成本地 TCP 服务。
 *
 *   DX-BT24  --BLE-->  本服务  --TCP 127.0.0.1:9000-->  chroot 里的 Python
 *
 * Python 侧用 pyserial 的 socket:// 直接连，协议不用改。
 */
public class BridgeService extends Service {

    static final String TAG = "BTBRIDGE";
    static final String TARGET_NAME = "bt24";
    /** 模块 MAC，作为名字匹配失败时的兜底 */
    static final String TARGET_MAC = "48:87:2D:D0:AC:A1";
    static final int TCP_PORT = 9000;

    static final UUID CCCD = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb");

    /** 给 Activity 显示状态用 */
    static volatile String status = "未启动";

    private BluetoothAdapter adapter;
    private BluetoothLeScanner scanner;
    private BluetoothGatt gatt;
    private BluetoothGattCharacteristic charWrite;
    private BluetoothGattCharacteristic charNotify;
    private int mtu = 23;

    private ServerSocket serverSocket;
    private volatile Socket client;
    private Thread tcpThread;
    private Thread supervisor;
    private volatile boolean running;
    private volatile boolean scanning;

    private static void L(String s) { Log.e(TAG, s); }

    @Override public void onCreate() {
        super.onCreate();
        startForeground(1, buildNotification("正在启动"));
        adapter = ((BluetoothManager) getSystemService(Context.BLUETOOTH_SERVICE)).getAdapter();
        running = true;
        startTcpServer();
        startSupervisor();
    }

    /**
     * 守护线程：每 3 秒检查一次，没连上且没在扫描就重新扫。
     * 蓝牙被关掉再打开、模块断电重启、连接丢失，都能自动恢复——
     * 比赛现场没人去点屏幕。
     */
    private void startSupervisor() {
        supervisor = new Thread(() -> {
            while (running) {
                try {
                    Thread.sleep(3000);
                } catch (InterruptedException e) {
                    break;
                }
                if (!running) break;
                if (gatt == null && !scanning) {
                    if (adapter != null && adapter.isEnabled()) {
                        startScan();
                    } else {
                        setStatus("蓝牙未开启");
                    }
                }
            }
        }, "supervisor");
        supervisor.setDaemon(true);
        supervisor.start();
    }

    @Override public int onStartCommand(Intent i, int flags, int startId) {
        return START_STICKY;
    }

    @Override public IBinder onBind(Intent i) { return null; }

    @Override public void onDestroy() {
        running = false;
        stopScan();
        if (gatt != null) { gatt.disconnect(); gatt.close(); }
        closeQuietly();
        super.onDestroy();
    }

    // ---------------------------------------------------------------- 前台通知

    private Notification buildNotification(String text) {
        NotificationManager nm = getSystemService(NotificationManager.class);
        if (Build.VERSION.SDK_INT >= 26) {
            nm.createNotificationChannel(
                new NotificationChannel("bridge", "BT 桥", NotificationManager.IMPORTANCE_LOW));
        }
        return new Notification.Builder(this, "bridge")
                .setContentTitle("BT 桥")
                .setContentText(text)
                .setSmallIcon(android.R.drawable.stat_sys_data_bluetooth)
                .build();
    }

    private void setStatus(String s) {
        status = s;
        L(s);
        NotificationManager nm = getSystemService(NotificationManager.class);
        nm.notify(1, buildNotification(s));
    }

    // ---------------------------------------------------------------- 扫描

    private boolean hasConnect() {
        return checkSelfPermission("android.permission.BLUETOOTH_CONNECT") == PackageManager.PERMISSION_GRANTED;
    }

    private boolean hasScan() {
        return checkSelfPermission("android.permission.BLUETOOTH_SCAN") == PackageManager.PERMISSION_GRANTED;
    }

    private void startScan() {
        if (adapter == null || !adapter.isEnabled()) { setStatus("蓝牙未开启"); return; }
        if (!hasScan()) { setStatus("缺少 BLUETOOTH_SCAN 权限，请用 adb 授权"); return; }
        scanner = adapter.getBluetoothLeScanner();
        if (scanner == null) { setStatus("扫描器不可用"); return; }
        setStatus("扫描 " + TARGET_NAME + " …");
        scanning = true;
        try {
            scanner.startScan(scanCallback);
        } catch (Exception e) {
            scanning = false;
            setStatus("启动扫描失败 " + e.getMessage());
        }
    }

    private void stopScan() {
        scanning = false;
        try { if (scanner != null) scanner.stopScan(scanCallback); } catch (Exception ignored) { }
    }

    private boolean isTarget(ScanResult r) {
        BluetoothDevice d = r.getDevice();
        String addr = d.getAddress();
        if (addr != null && addr.equalsIgnoreCase(TARGET_MAC)) return true;
        if (!hasConnect()) return false;
        String n = null;
        try { n = d.getName(); } catch (Exception ignored) { }
        if (n == null) { try { n = r.getScanRecord() != null ? r.getScanRecord().getDeviceName() : null; } catch (Exception ignored) { } }
        return n != null && n.equalsIgnoreCase(TARGET_NAME);
    }

    private final ScanCallback scanCallback = new ScanCallback() {
        @Override public void onScanResult(int type, ScanResult result) {
            if (!isTarget(result)) return;
            if (gatt != null) return;
            L("找到目标 " + result.getDevice().getAddress()
              + " rssi=" + result.getRssi());
            stopScan();
            setStatus("连接中 " + result.getDevice().getAddress());
            connect(result.getDevice());
        }

        @Override public void onScanFailed(int errorCode) {
            setStatus("扫描失败 code=" + errorCode);
        }
    };

    // ---------------------------------------------------------------- GATT

    private void connect(BluetoothDevice dev) {
        if (!hasConnect()) { setStatus("缺少 BLUETOOTH_CONNECT 权限"); return; }
        gatt = dev.connectGatt(this, false, gattCallback, BluetoothDevice.TRANSPORT_LE);
    }

    private final BluetoothGattCallback gattCallback = new BluetoothGattCallback() {

        @Override public void onConnectionStateChange(BluetoothGatt g, int status, int newState) {
            L("连接状态 status=" + status + " newState=" + newState);
            if (newState == BluetoothGatt.STATE_CONNECTED) {
                setStatus("已连接，协商 MTU");
                g.requestMtu(247);
            } else if (newState == BluetoothGatt.STATE_DISCONNECTED) {
                g.close();
                gatt = null;
                charWrite = charNotify = null;
                // 不在这里自己重连：交给 supervisor 统一处理，避免重复扫描
                setStatus("断开，3 秒后自动重连");
            }
        }

        @Override public void onMtuChanged(BluetoothGatt g, int m, int status) {
            mtu = m;
            L("MTU = " + m + " status=" + status);
            g.discoverServices();
        }

        @Override public void onServicesDiscovered(BluetoothGatt g, int status) {
            L("服务发现 status=" + status);
            dumpGatt(g);
            pickCharacteristics(g);
            if (charWrite != null && charNotify != null) {
                enableNotify(g, charNotify);
            } else {
                setStatus("没找到可用的读写特征值");
            }
        }

        @Override public void onDescriptorWrite(BluetoothGatt g, BluetoothGattDescriptor d, int status) {
            L("描述符写入 status=" + status);
            if (status == BluetoothGatt.GATT_SUCCESS) {
                setStatus("就绪：" + (client != null ? "TCP 已连" : "等待 TCP 连接 :" + TCP_PORT));
            }
        }

        @Override public void onCharacteristicChanged(BluetoothGatt g, BluetoothGattCharacteristic ch, byte[] value) {
            onBtData(value);
        }

        // API 32 及以下的旧回调
        @Override public void onCharacteristicChanged(BluetoothGatt g, BluetoothGattCharacteristic ch) {
            if (Build.VERSION.SDK_INT < 33) onBtData(ch.getValue());
        }
    };

    /** 把 GATT 表打到 logcat，方便确认 UUID */
    private void dumpGatt(BluetoothGatt g) {
        try {
            List<BluetoothGattService> services = g.getServices();
            L("==== GATT 表 共 " + services.size() + " 个 service ====");
            for (BluetoothGattService s : services) {
                L("SERVICE " + s.getUuid());
                for (BluetoothGattCharacteristic c : s.getCharacteristics()) {
                    L("  CHAR " + c.getUuid() + " props=0x" + Integer.toHexString(c.getProperties()));
                }
            }
            L("==== GATT 表结束 ====");
        } catch (Exception e) {
            L("dump 失败 " + e);
        }
    }

    private void pickCharacteristics(BluetoothGatt g) {
        for (BluetoothGattService s : g.getServices()) {
            for (BluetoothGattCharacteristic c : s.getCharacteristics()) {
                int p = c.getProperties();
                boolean w = (p & (BluetoothGattCharacteristic.PROPERTY_WRITE
                        | BluetoothGattCharacteristic.PROPERTY_WRITE_NO_RESPONSE)) != 0;
                boolean n = (p & BluetoothGattCharacteristic.PROPERTY_NOTIFY) != 0;
                if (w && charWrite == null) charWrite = c;
                if (n && charNotify == null) charNotify = c;
            }
        }
        if (charWrite != null) L("选中写入 " + charWrite.getUuid());
        if (charNotify != null) L("选中通知 " + charNotify.getUuid());
    }

    private void enableNotify(BluetoothGatt g, BluetoothGattCharacteristic c) {
        g.setCharacteristicNotification(c, true);
        BluetoothGattDescriptor d = c.getDescriptor(CCCD);
        if (d != null) {
            if (Build.VERSION.SDK_INT >= 33) {
                g.writeDescriptor(d, BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE);
            } else {
                d.setValue(BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE);
                g.writeDescriptor(d);
            }
        } else {
            setStatus("没有 CCCD，跳过通知配置");
        }
    }

    // ---------------------------------------------------------------- TCP

    private volatile OutputStream tcpOut;

    private void startTcpServer() {
        tcpThread = new Thread(() -> {
            ServerSocket ss;
            try {
                ss = new ServerSocket(TCP_PORT, 1, InetAddress.getByName("127.0.0.1"));
                serverSocket = ss;
                L("TCP 监听 127.0.0.1:" + TCP_PORT);
            } catch (Exception e) {
                setStatus("TCP 端口占用 " + e.getMessage());
                return;
            }

            // 注意：每个客户端单独 try/catch。
            // 之前把异常抛到外层，客户端一断开就会打死整个 accept 循环，
            // 之后所有新连接都进不来，tcpOut 还指着死掉的 socket。
            while (running) {
                Socket s;
                try {
                    s = ss.accept();
                } catch (Exception e) {
                    if (!running) break;
                    L("accept 失败: " + e);
                    try { Thread.sleep(200); } catch (InterruptedException ignored) { }
                    continue;
                }

                try {
                    L("TCP 客户端接入 " + s.getRemoteSocketAddress());
                    closeClient();
                    client = s;
                    s.setTcpNoDelay(true);
                    tcpOut = s.getOutputStream();
                    InputStream in = s.getInputStream();
                    byte[] buf = new byte[1024];
                    while (running) {
                        int n = in.read(buf);
                        if (n < 0) break;
                        sendToBt(buf, n);
                    }
                } catch (Exception e) {
                    if (running) L("客户端断开: " + e.getMessage());
                } finally {
                    closeClient();
                    L("等待下一个 TCP 连接");
                }
            }
        }, "tcp-server");
        tcpThread.start();
    }

    private void closeClient() {
        Socket s = client;
        client = null;
        tcpOut = null;
        if (s != null) try { s.close(); } catch (Exception ignored) { }
    }

    private void closeQuietly() {
        closeClient();
        if (serverSocket != null) try { serverSocket.close(); } catch (Exception ignored) { }
    }

    /** TCP -> BLE，按 (MTU-3) 分包 */
    private synchronized void sendToBt(byte[] data, int len) {
        BluetoothGattCharacteristic c = charWrite;
        if (c == null || gatt == null) return;
        int chunk = Math.max(20, mtu - 3);
        for (int off = 0; off < len; off += chunk) {
            int n = Math.min(chunk, len - off);
            byte[] part = new byte[n];
            System.arraycopy(data, off, part, 0, n);
            try {
                if (Build.VERSION.SDK_INT >= 33) {
                    gatt.writeCharacteristic(c, part, BluetoothGattCharacteristic.WRITE_TYPE_NO_RESPONSE);
                } else {
                    c.setWriteType(BluetoothGattCharacteristic.WRITE_TYPE_NO_RESPONSE);
                    c.setValue(part);
                    gatt.writeCharacteristic(c);
                }
            } catch (Exception e) {
                L("写 BLE 失败 " + e);
            }
        }
    }

    /** BLE -> TCP */
    private void onBtData(byte[] value) {
        if (value == null || value.length == 0) return;
        OutputStream o = tcpOut;
        if (o == null) return;
        try {
            o.write(value);
            o.flush();
        } catch (Exception e) {
            // 对端已经断了，清掉引用，等下一个客户端接入，别在这里刷日志
            tcpOut = null;
        }
    }
}
