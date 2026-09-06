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
