package com.robot.btbridge;

import android.app.Activity;
import android.content.Intent;
import android.os.Build;
import android.os.Bundle;
import android.widget.TextView;

/** 只负责申请权限、拉起前台服务、显示状态。真正的活都在 BridgeService 里。 */
public class MainActivity extends Activity {

    private TextView tv;

    @Override protected void onCreate(Bundle b) {
        super.onCreate(b);
        tv = new TextView(this);
        tv.setTextSize(18f);
        tv.setPadding(48, 96, 48, 48);
        tv.setText("启动中…");
        setContentView(tv);

        if (Build.VERSION.SDK_INT >= 31) {
            requestPermissions(new String[]{
                    "android.permission.BLUETOOTH_SCAN",
                    "android.permission.BLUETOOTH_CONNECT",
                    "android.permission.POST_NOTIFICATIONS"
            }, 1);
        }
    }

    @Override public void onRequestPermissionsResult(int code, String[] p, int[] r) {
        super.onRequestPermissionsResult(code, p, r);
        startBridge();
    }

    @Override protected void onResume() {
        super.onResume();
        startBridge();
        tv.postDelayed(new Runnable() {
            @Override public void run() {
                tv.setText(BridgeService.status + "\n\n127.0.0.1:9000");
                tv.postDelayed(this, 500);
            }
        }, 500);
    }

    private void startBridge() {
        Intent i = new Intent(this, BridgeService.class);
        if (Build.VERSION.SDK_INT >= 26) startForegroundService(i); else startService(i);
    }
}
