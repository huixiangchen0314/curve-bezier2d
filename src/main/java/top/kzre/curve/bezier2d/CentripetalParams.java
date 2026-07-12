package top.kzre.curve.bezier2d;

/**
 * 向心参数化（Centripetal parameterization）工具类。
 * 根据点序列生成参数 t，使得拟合曲线能更好地捕捉尖锐细节。
 */
public final class CentripetalParams {
    private CentripetalParams() {}

    /**
     * 计算向心参数 t 数组，长度与点数量相同，归一化到 [0, 1]。
     * @param xs    x 坐标数组
     * @param ys    y 坐标数组
     * @param alpha 指数因子，通常为 0.5（向心），1.0 为弦长参数化，0.0 为均匀参数化
     * @return 归一化的参数 t 数组，t[0]=0, t[n-1]=1
     */
    public static double[] compute(double[] xs, double[] ys, double alpha) {
        int n = xs.length;
        double[] t = new double[n];
        t[0] = 0.0;
        for (int i = 1; i < n; i++) {
            double dx = xs[i] - xs[i - 1];
            double dy = ys[i] - ys[i - 1];
            double d = Math.sqrt(dx * dx + dy * dy);
            t[i] = t[i - 1] + Math.pow(d, alpha);
        }
        double total = t[n - 1];
        if (total > 0) {
            for (int i = 0; i < n; i++) {
                t[i] /= total;
            }
        } else {
            // 所有点重合，退化为均匀参数化
            for (int i = 0; i < n; i++) {
                t[i] = (double) i / (n - 1);
            }
        }
        return t;
    }

    /**
     * 使用默认向心参数（alpha = 0.5）。
     */
    public static double[] centripetal(double[] xs, double[] ys) {
        return compute(xs, ys, 0.5);
    }

    /**
     * 弦长参数化（alpha = 1.0），与 ChordLengthTable 结果一致。
     */
    public static double[] chordal(double[] xs, double[] ys) {
        return compute(xs, ys, 1.0);
    }
}