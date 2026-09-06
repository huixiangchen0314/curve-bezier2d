package top.kzre.curve.bezier2d;

public final class ChordLengthUtils {

    private ChordLengthUtils() {}

    /**
     * 根据曲线和给定的参数 t 列表，计算对应的累积弦长 s 参数。
     * @param curve   曲线
     * @param tParams 参数数组（需递增，范围 [0,1]）
     * @return 累积弦长数组，与 tParams 长度相同
     */
    public static double[] buildChordLengthParams(Curve curve, double[] tParams) {
        int n = tParams.length;
        double[] sParams = new double[n];
        double prevX = 0, prevY = 0;
        for (int i = 0; i < n; i++) {
            double t = tParams[i];
            Pair p = Bezier2D.eval(curve, t);
            if (i == 0) {
                sParams[i] = 0.0;
                prevX = p.getX();
                prevY = p.getY();
            } else {
                double dx = p.getX() - prevX;
                double dy = p.getY() - prevY;
                sParams[i] = sParams[i - 1] + Math.hypot(dx, dy);
                prevX = p.getX();
                prevY = p.getY();
            }
        }
        return sParams;
    }

    /**
     * 从曲线均匀采样构建弦长参数映射表。
     * @param curve   曲线
     * @param samples 采样点数（≥2）
     * @return TableMapping，包含 t 和累积弦长
     */
    public static TableMapping sample(Curve curve, int samples) {
        double[] tParams = new double[samples];
        for (int i = 0; i < samples; i++) {
            tParams[i] = (double) i / (samples - 1);
        }
        return sample(curve, tParams);
    }

    /**
     * 从曲线和指定的 t 参数列表构建弦长参数映射表。
     * @param curve   曲线
     * @param tParams 参数数组（递增）
     * @return TableMapping，包含 t 和累积弦长
     */
    public static TableMapping sample(Curve curve, double[] tParams) {
        double[] sParams = buildChordLengthParams(curve, tParams);
        return new TableMapping(tParams, sParams);
    }

    public static TableMapping fromPoints(double[] xs, double[] ys){
        if (xs == null || ys == null || xs.length != ys.length || xs.length < 2) {
            throw new IllegalArgumentException("Arrays must be non‑null, equal length and at least 2 points");
        }
        int n = xs.length;
        double[] tParams = new double[n];
        double[] sParams = new double[n];
        for (int i = 0; i < n; i++) {
            tParams[i] = (double) i / (n - 1);
        }
        sParams[0] = 0;
        double cum = 0;
        for (int i = 1; i < n; i++) {
            double dx = xs[i] - xs[i - 1];
            double dy = ys[i] - ys[i - 1];
            cum += Math.hypot(dx, dy);
            sParams[i] = cum;
        }
        return new TableMapping(tParams, sParams);
    }
}