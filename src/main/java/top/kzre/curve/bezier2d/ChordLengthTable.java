package top.kzre.curve.bezier2d;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * 弦长表：基于采样点间的直线距离累积，建立参数 t 与弦长 s 的双向映射。
 * 可直接从点序列或从曲线采样构建，用于快速弧长近似或拟合时的弦长参数化。
 */
public class ChordLengthTable {

    private final double[] tValues;       // 采样点的均匀参数 t（基于索引）
    private final double[] chordLengths;  // 累积弦长

    /**
     * 从坐标数组构建弦长表。
     * @param xs x 坐标数组
     * @param ys y 坐标数组
     */
    public ChordLengthTable(double[] xs, double[] ys) {
        if (xs == null || ys == null || xs.length != ys.length || xs.length < 2)
            throw new IllegalArgumentException("Arrays must be non‑null, equal length and at least 2 points");
        int n = xs.length;
        tValues = new double[n];
        chordLengths = new double[n];
        for (int i = 0; i < n; i++) {
            tValues[i] = (double) i / (n - 1);
        }
        chordLengths[0] = 0;
        double cum = 0;
        for (int i = 1; i < n; i++) {
            double dx = xs[i] - xs[i - 1];
            double dy = ys[i] - ys[i - 1];
            cum += Math.hypot(dx, dy);
            chordLengths[i] = cum;
        }
    }

    /**
     * 从曲线均匀采样构建弦长表。
     * @param curve   目标曲线
     * @param samples 采样点个数（≥2）
     */
    public ChordLengthTable(Curve curve, int samples) {
        if (samples < 2) throw new IllegalArgumentException("samples must be >= 2");
        List<Pair> pts = new ArrayList<>(samples);
        for (int i = 0; i < samples; i++) {
            double t = (double) i / (samples - 1);
            pts.add(Bezier2D.eval(curve, t));
        }
        tValues = new double[samples];
        chordLengths = new double[samples];
        for (int i = 0; i < samples; i++) {
            tValues[i] = (double) i / (samples - 1);
        }
        chordLengths[0] = 0;
        double cum = 0;
        for (int i = 1; i < samples; i++) {
            Pair p0 = pts.get(i - 1);
            Pair p1 = pts.get(i);
            cum += Math.hypot(p1.getX() - p0.getX(), p1.getY() - p0.getY());
            chordLengths[i] = cum;
        }
    }

    /** 总弦长 */
    public double totalLength() {
        return chordLengths[chordLengths.length - 1];
    }

    /**
     * 获取每个原始点的归一化弦长参数 t。
     * 数组长度与构造时输入的点数相同，首尾值为 0 和 1。
     */
    public double[] getParameters() {
        int n = chordLengths.length;
        double total = totalLength();
        double[] t = new double[n];
        for (int i = 0; i < n; i++) {
            t[i] = chordLengths[i] / total;
        }
        t[0] = 0.0;
        t[n - 1] = 1.0;
        return t;
    }



    /** 给定参数 t ∈ [0,1]，返回对应的弦长 s */
    public double getLength(double t) {
        if (t <= 0) return 0;
        if (t >= 1) return totalLength();
        int idx = intervalIndex(tValues, t);
        double t0 = tValues[idx];
        double t1 = tValues[idx + 1];
        double frac = (t - t0) / (t1 - t0);
        return chordLengths[idx] + frac * (chordLengths[idx + 1] - chordLengths[idx]);
    }

    /** 给定弦长 s，返回对应的参数 t */
    public double getT(double s) {
        if (s <= 0) return 0;
        if (s >= totalLength()) return 1;
        int idx = intervalIndex(chordLengths, s);
        double s0 = chordLengths[idx];
        double s1 = chordLengths[idx + 1];
        double frac = (s - s0) / (s1 - s0);
        return tValues[idx] + frac * (tValues[idx + 1] - tValues[idx]);
    }

    /** 使用二分查找确定 key 所在的区间 [i, i+1]，满足 arr[i] <= key <= arr[i+1] */
    private int intervalIndex(double[] arr, double key) {
        int pos = Arrays.binarySearch(arr, key);
        if (pos >= 0) {
            return Math.min(pos, arr.length - 2);
        } else {
            int low = -pos - 2;
            if (low < 0) return 0;
            if (low > arr.length - 2) return arr.length - 2;
            return low;
        }
    }
}