package top.kzre.curve.bezier2d;

import lombok.Getter;

import java.util.Arrays;

/**
 * 基于表的参数映射
 */
public final class TableMapping implements ParamMapping {
    @Getter
    private final double[] tParams;
    @Getter
    private final double[] sParams;
    @Getter
    private final int samples;
    @Getter
    private final double maxS;


    /**
     * 将累积长度 s 归一化到 [0,1]，返回新数组。
     * 若 sParams 为空或长度为零，返回空数组。
     * 若最大值为零（全零），返回全零数组。
     */
    public static double[] uniformSParams(double[] sParams) {
        if (sParams == null || sParams.length == 0) {
            return new double[0];
        }
        double[] out = new double[sParams.length];
        uniformSParams(sParams, out);
        return out;
    }

    /**
     * 将累积长度 s 归一化到 [0,1]，结果存入 out。
     * out 长度必须与 sParams 相同。
     */
    public static void uniformSParams(double[] sParams, double[] out) {
        if (sParams == null || out == null || sParams.length == 0 || out.length != sParams.length) {
            throw new IllegalArgumentException("sParams and out must be non-null, same length, and non-empty");
        }
        double max = sParams[sParams.length - 1];
        if (max == 0) {
            Arrays.fill(out, 0.0);
            return;
        }
        for (int i = 0; i < sParams.length; i++) {
            out[i] = sParams[i] / max;
        }
        // 确保首尾精确
        out[0] = 0.0;
        out[out.length - 1] = 1.0;
    }


    public TableMapping(double[] tParams, double[] sParams) {
        samples = tParams.length;
        this.tParams = tParams;
        this.sParams = sParams;
        maxS = sParams[samples - 1];
    }


    /** 给定参数 t ∈ [0,1]，返回对应的弦长 s */
    @Override
    public double getS(double t) {
        if (t <= 0) return 0;
        if (t >= 1) return maxS;
        int idx = intervalIndex(tParams, t);
        double t0 = tParams[idx];
        double t1 = tParams[idx + 1];
        double frac = (t - t0) / (t1 - t0);
        return sParams[idx] + frac * (sParams[idx + 1] - sParams[idx]);
    }

    /** 给定弦长 s，返回对应的参数 t */
    @Override
    public double getT(double s) {
        if (s <= 0) return 0;
        if (s >= maxS) return 1;
        int idx = intervalIndex(sParams, s);
        double s0 = sParams[idx];
        double s1 = sParams[idx + 1];
        double frac = (s - s0) / (s1 - s0);
        return tParams[idx] + frac * (tParams[idx + 1] - tParams[idx]);
    }

    /** 使用二分查找确定 key 所在的区间 [i, i+1]，满足 arr[i] <= key <= arr[i+1] */
    private int intervalIndex(double[] arr, double key) {
        int pos = Arrays.binarySearch(arr, key);
        if (pos >= 0) {
            return Math.min(pos, arr.length - 2);
        } else {
            int low = -pos - 2;
            if (low < 0) return 0;
            return Math.min(low, arr.length - 2);
        }
    }

}
