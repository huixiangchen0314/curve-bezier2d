package top.kzre.curve.bezier2d;

public final class ArcLengthUtils {

    private ArcLengthUtils() {}

    /**
     * 计算曲线在参数区间 [ta, tb] 上的弧长，使用 Simpson 积分。
     */
    public static double integrateArcLength(Curve curve, double ta, double tb, int subdiv) {
        if (subdiv % 2 != 0) subdiv++;
        double h = (tb - ta) / subdiv;
        double sum = speed(curve, ta) + speed(curve, tb);
        for (int i = 1; i < subdiv; i++) {
            double t = ta + i * h;
            sum += (i % 2 == 0 ? 2 : 4) * speed(curve, t);
        }
        return (h / 3.0) * sum;
    }

    /**
     * 计算曲线在参数 t 处的速度（一阶导数的模长）。
     */
    public static double speed(Curve curve, double t) {
        Pair d = Bezier2D.deriv(curve, t);
        return Math.hypot(d.getX(), d.getY());
    }

    /**
     * 从曲线采样构建弧长参数表（返回 t 和累积弧长数组）。
     */
    public static double[] buildArcLengthParams(Curve curve, double[] tParams) {
        int samples = tParams.length;
        double[] arcParams = new double[samples];
        buildArcLengthParams(curve, tParams, arcParams);
        return arcParams;
    }

    public static void buildArcLengthParams(Curve curve, double[] tParams, double[] arcParams) {
        assert arcParams.length == tParams.length;

        int samples = tParams.length;
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
    }

    public static TableMapping sample(Curve curve, int samples) {
        double[] tParams = new double[samples];

        for (int i = 0; i < samples; i++) {
            tParams[i] = (double) i / (samples - 1);
        }
        return sample(curve, tParams);
    }

    public static TableMapping sample(Curve curve, double[] tParams) {
        double[] arcParams =  buildArcLengthParams(curve, tParams);
        return new TableMapping(tParams, arcParams);
    }

}