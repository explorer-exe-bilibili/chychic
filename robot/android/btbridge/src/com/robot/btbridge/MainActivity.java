package com.robot.btbridge;

import android.app.Activity;
import android.content.Intent;
import android.graphics.Color;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.view.ViewGroup;
import android.widget.LinearLayout;
import android.widget.TextView;

/**
 * 界面：顶部状态行 + 中间实时波形。
 *
 * 波形数据来自 Service 解析出的 FB_STATE，与通过 TCP 发给上位机的是同一份字节，
 * 所以这里看到的和上位机看到的一致。
 */
public class MainActivity extends Activity {

    private TextView status;
    private TextView legend;
    private ChartView chart;
    private final Handler handler = new Handler(Looper.getMainLooper());

    private final Runnable tick = new Runnable() {
        @Override public void run() {
            refresh();
            handler.postDelayed(this, 50);
        }
    };

    @Override protected void onCreate(Bundle b) {
        super.onCreate(b);

        LinearLayout root = new LinearLayout(this);
        root.setOrientation(LinearLayout.VERTICAL);
        root.setBackgroundColor(0xFF000000);

        status = new TextView(this);
        status.setTextSize(13f);
        status.setTextColor(0xFFDDDDDD);
        status.setPadding(24, 36, 24, 12);
        status.setTypeface(android.graphics.Typeface.MONOSPACE);

        legend = new TextView(this);
        legend.setTextSize(11f);
        legend.setTextColor(0xFF777777);
        legend.setPadding(24, 0, 24, 8);
        legend.setText("127.0.0.1:9000  ·  白=端口待连");

        chart = new ChartView(this);
        chart.setLayoutParams(new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, 0, 1f));

        root.addView(status);
        root.addView(legend);
        root.addView(chart);
        setContentView(root);

        applyEdgeToEdge(root);

        if (Build.VERSION.SDK_INT >= 31) {
            requestPermissions(new String[]{
                    "android.permission.BLUETOOTH_SCAN",
                    "android.permission.BLUETOOTH_CONNECT",
                    "android.permission.POST_NOTIFICATIONS"
            }, 1);
        }
    }

    @SuppressWarnings("deprecation")
    private void applyEdgeToEdge(LinearLayout root) {
        if (Build.VERSION.SDK_INT >= 21) {
            getWindow().setStatusBarColor(Color.parseColor("#111111"));
            getWindow().setNavigationBarColor(Color.parseColor("#111111"));
        }
        root.setFitsSystemWindows(true);
    }

    @Override public void onRequestPermissionsResult(int code, String[] p, int[] r) {
        super.onRequestPermissionsResult(code, p, r);
        startBridge();
    }

    @Override protected void onResume() {
        super.onResume();
        startBridge();
        handler.post(tick);
    }

    @Override protected void onPause() {
        super.onPause();
        handler.removeCallbacks(tick);
    }

    private void startBridge() {
        Intent i = new Intent(this, BridgeService.class);
        if (Build.VERSION.SDK_INT >= 26) startForegroundService(i); else startService(i);
    }

    private void refresh() {
        Proto.State s = LiveData.latest;
        boolean online = LiveData.online();

        if (s == null) {
            status.setText((BridgeService.status == null ? "启动中" : BridgeService.status)
                    + "\n等待下位机数据…");
        } else {
            String mode;
            switch (s.mode) {
                case Proto.MODE_LINE_FOLLOW: mode = "循迹"; break;
                case Proto.MODE_MANUAL:      mode = "手动"; break;
                case Proto.MODE_ESTOP:       mode = "急停"; break;
                default:                     mode = "待机"; break;
            }
            StringBuilder sb = new StringBuilder();
            sb.append(online ? "● 在线" : "○ 断开")
              .append("   ").append(mode)
              .append("   循迹=").append(lfName(s.lfState))
              .append("   线=").append(mask(s.lineMask));
            if (s.watchdog()) sb.append("   [看门狗]");
            if (s.estop()) sb.append("   [急停]");
            sb.append("\nL=").append(fmt(s.vLeft)).append("   R=").append(fmt(s.vRight))
              .append("   目标=").append(fmt(s.vCmd))
              .append("   转向=").append(String.format("%+.2f", s.wCmd));
            sb.append("\n帧 ").append(LiveData.bleFrames)
              .append("   校验错 ").append(LiveData.bleCrcErrors)
              .append("   enc ").append(s.encLeft).append("/").append(s.encRight);
            status.setText(sb.toString());
        }

        Proto.LiftState l = LiveData.latestLift;
        String lift = l == null ? "升降 无数据"
                : String.format("升降 %.1fmm → %dmm %s",
                    l.positionMm, l.targetMm, l.referenced() ? "" : "(未标定)");
        legend.setText("127.0.0.1:9000  ·  " + lift);
        chart.invalidate();
    }

    private static String fmt(float v) {
        return String.format("%+6.0f", v);
    }

    private static String lfName(int v) {
        switch (v) {
            case 1: return "找线";
            case 2: return "跟踪";
            case 3: return "丢线";
            case 4: return "宽线";
            case 5: return "歧义";
            default: return "未启用";
        }
    }

    private static String mask(int m) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < 5; i++) sb.append(((m >> i) & 1) == 1 ? '1' : '0');
        return sb.toString();
    }
}
