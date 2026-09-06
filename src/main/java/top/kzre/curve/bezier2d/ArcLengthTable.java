package top.kzre.curve.bezier2d;

import java.util.Arrays;

@Deprecated
public final class ArcLengthTable {
    private final double[] tValues;
    private final double[] arcLengths;
    private final double curveLength;

    public ArcLengthTable(double[] tParams, double[] arcParams) {
        if (tParams.length < 2) throw new IllegalArgumentException("samples must be at least 2");
        this.tValues = tParams;
        this.arcLengths = arcParams;
        int size = tValues.length;
        curveLength = arcLengths[size -1];
    }


    // ---------- 公共接口 ----------
    public double totalLength() {
        return curveLength;
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

    /**
     * 采样曲线构建弦长表
     */
    public static ArcLengthTable sample(Curve curve, int samples) {
        double[] tParams = new double[samples];
        double[] arcParams = new double[samples];
        for (int i = 0; i < samples; i++) {
            tParams[i] = (double) i / (samples - 1);
        }
        arcParams[0] = 0.0;
        double cum = 0.0;
        for (int i = 0; i < samples - 1; i++) {
            double t0 = tParams[i];
            double t1 = tParams[i + 1];
            double segLen = integrateArcLength(curve, t0, t1, 8);
            cum += segLen;
            arcParams[i + 1] = cum;
        }
        return new  ArcLengthTable(tParams, arcParams);
    }

    private static double integrateArcLength(Curve curve, double ta, double tb, int subdiv) {
        if (subdiv % 2 != 0) subdiv++;
        double h = (tb - ta) / subdiv;
        double sum = speed(curve, ta) + speed(curve, tb);
        for (int i = 1; i < subdiv; i++) {
            double t = ta + i * h;
            sum += (i % 2 == 0 ? 2 : 4) * speed(curve, t);
        }
        return (h / 3.0) * sum;
    }

    private static double speed(Curve curve, double t) {
        Pair d = Bezier2D.deriv(curve, t);   // 通过实例调用 deriv
        return Math.hypot(d.getX(), d.getY());
    }
}