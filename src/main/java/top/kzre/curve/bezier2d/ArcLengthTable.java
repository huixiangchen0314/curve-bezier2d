package top.kzre.curve.bezier2d;

import java.util.Arrays;

public final class ArcLengthTable {
    private final Bezier2D.Spec bezier = new Bezier2DImpl();  // 用于求导
    private final Curve curve;
    private final int samples;
    private final double[] tValues;
    private final double[] arcLengths;

    public ArcLengthTable(Curve curve, int samples) {
        if (samples < 2) throw new IllegalArgumentException("samples must be at least 2");
        this.curve = curve;
        this.samples = samples;
        this.tValues = new double[samples];
        this.arcLengths = new double[samples];
        buildTable();
    }

    private void buildTable() {
        for (int i = 0; i < samples; i++) {
            tValues[i] = (double) i / (samples - 1);
        }
        arcLengths[0] = 0.0;
        double cum = 0.0;
        for (int i = 0; i < samples - 1; i++) {
            double t0 = tValues[i];
            double t1 = tValues[i + 1];
            double segLen = integrateArcLength(t0, t1, 8);
            cum += segLen;
            arcLengths[i + 1] = cum;
        }
    }

    private double integrateArcLength(double ta, double tb, int subdiv) {
        if (subdiv % 2 != 0) subdiv++;
        double h = (tb - ta) / subdiv;
        double sum = speed(ta) + speed(tb);
        for (int i = 1; i < subdiv; i++) {
            double t = ta + i * h;
            sum += (i % 2 == 0 ? 2 : 4) * speed(t);
        }
        return (h / 3.0) * sum;
    }

    private double speed(double t) {
        Pair d = bezier.deriv(curve, t);   // 通过实例调用 deriv
        return Math.hypot(d.getX(), d.getY());
    }

    // ---------- 公共接口 ----------
    public double totalLength() {
        return arcLengths[samples - 1];
    }

    public double getLength(double t) {
        if (t <= 0) return 0;
        if (t >= 1) return totalLength();
        int idx = intervalIndex(tValues, t);
        double t0 = tValues[idx];
        double t1 = tValues[idx + 1];
        double frac = (t - t0) / (t1 - t0);
        return arcLengths[idx] + frac * (arcLengths[idx + 1] - arcLengths[idx]);
    }

    public double getT(double s) {
        if (s <= 0) return 0;
        if (s >= totalLength()) return 1;
        int idx = intervalIndex(arcLengths, s);
        double s0 = arcLengths[idx];
        double s1 = arcLengths[idx + 1];
        double frac = (s - s0) / (s1 - s0);
        return tValues[idx] + frac * (tValues[idx + 1] - tValues[idx]);
    }

    /**
     * 使用 Arrays.binarySearch 查找区间索引 i，满足 arr[i] <= key <= arr[i+1]。
     * 若 key 与 arr 中某元素相等，返回该索引（此时区间为 [i, i]，调用者需保证 i+1 存在）。
     */
    private int intervalIndex(double[] arr, double key) {
        int pos = Arrays.binarySearch(arr, key);
        if (pos >= 0) {
            // 精确匹配，取 pos，若 pos 为最后一个元素，则返回前一个区间以避免越界
            return Math.min(pos, arr.length - 2);
        } else {
            // 未找到，插值点 = -(pos + 1)，所以 low = -(pos+2) 是最后一个 ≤ key 的索引
            int low = -pos - 2;
            // 保证 low 在有效范围 [0, arr.length-2]
            if (low < 0) return 0;
            return Math.min(low, arr.length - 2);
        }
    }
}