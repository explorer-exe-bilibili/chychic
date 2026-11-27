package com.robot.btbridge;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Path;
import android.view.View;

/** 实时波形：左轮实测 / 右轮实测 / 目标速度。 */
public class ChartView extends View {

    private final Paint paint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint grid = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint text = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Path path = new Path();

    private static final int C_LEFT = 0xFFFF9800;   // 橙：左轮
    private static final int C_RIGHT = 0xFF29B6F6;  // 蓝：右轮
    private static final int C_CMD = 0xFF9E9E9E;    // 灰：目标

    private final float[] vl = new float[LiveData.CAP];
    private final float[] vr = new float[LiveData.CAP];
    private final float[] cmd = new float[LiveData.CAP];

    private static final int PAD_L = 52, PAD_R = 10, PAD_T = 26, PAD_B = 20;

    public ChartView(Context c) {
        super(c);
        grid.setColor(0xFF2A2A2A);
        grid.setStrokeWidth(1f);
        text.setColor(0xFF9E9E9E);
        text.setTextSize(getResources().getDisplayMetrics().scaledDensity * 10);
        setBackgroundColor(0xFF121212);
    }

    @Override protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        int w = getWidth(), h = getHeight();
        float plotW = w - PAD_L - PAD_R;
        float plotH = h - PAD_T - PAD_B;
        if (plotW <= 0 || plotH <= 0) return;

        int n = LiveData.size();
        LiveData.snapshot(vl, vr, cmd);

        // Y 轴范围：对称，最少 ±120 mm/s
        float maxAbs = 120f;
        for (int i = 0; i < n; i++) {
            maxAbs = Math.max(maxAbs, Math.abs(vl[i]));
            maxAbs = Math.max(maxAbs, Math.abs(vr[i]));
            maxAbs = Math.max(maxAbs, Math.abs(cmd[i]));
        }
        maxAbs = (float) (Math.ceil(maxAbs / 100.0) * 100.0);

        float midY = PAD_T + plotH / 2f;
        float scale = plotH / 2f / maxAbs;

        // 网格与刻度
        for (int k = -1; k <= 1; k++) {
            float y = midY - k * maxAbs * scale;
            grid.setColor(k == 0 ? 0xFF3A3A3A : 0xFF232323);
            canvas.drawLine(PAD_L, y, w - PAD_R, y, grid);
            canvas.drawText(String.valueOf((int) (k * maxAbs)), 6, y + 4, text);
        }

        // 单位放在右上角，避免和顶部刻度打架
        canvas.drawText("mm/s", w - PAD_R - 52, PAD_T - 12, text);

        if (n < 2) {
            canvas.drawText("等待数据…", PAD_L + 12, midY, text);
            return;
        }

        drawTrace(canvas, cmd, n, plotW, midY, scale, C_CMD, true);
        drawTrace(canvas, vl, n, plotW, midY, scale, C_LEFT, false);
        drawTrace(canvas, vr, n, plotW, midY, scale, C_RIGHT, false);

        // 图例
        text.setColor(C_LEFT);   canvas.drawText("左轮", PAD_L + 4, 16, text);
        text.setColor(C_RIGHT);  canvas.drawText("右轮", PAD_L + 52, 16, text);
        text.setColor(C_CMD);    canvas.drawText("目标", PAD_L + 100, 16, text);
        text.setColor(0xFF9E9E9E);
    }

    private void drawTrace(Canvas canvas, float[] data, int n, float plotW, float midY,
                           float scale, int color, boolean dashed) {
        paint.setColor(color);
        paint.setStrokeWidth(dashed ? 1.5f : 2.2f);
        paint.setStyle(Paint.Style.STROKE);

        path.reset();
        for (int i = 0; i < n; i++) {
            float x = PAD_L + plotW * i / (float) (n - 1);
            float y = midY - Math.max(-1e4f, Math.min(1e4f, data[i])) * scale;
            if (i == 0) path.moveTo(x, y); else path.lineTo(x, y);
        }
        canvas.drawPath(path, paint);
    }
}
