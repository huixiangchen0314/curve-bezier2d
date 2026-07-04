package top.kzre.curve.bezier2d;

import java.util.Arrays;

public final class CurveFitter {
    private CurveFitter() {}

    public static Curve fit(double[] xs, double[] ys, double maxError, int maxSeg) {
        if (xs == null || ys == null || xs.length != ys.length || xs.length < 2)
            throw new IllegalArgumentException("Invalid input arrays");
        ChordLengthTable table = new ChordLengthTable(xs, ys);
        double[] t = table.getParameters();
        return fitRecursive(xs, ys, t, maxError, maxSeg);
    }

    private static Curve fitRecursive(double[] xs, double[] ys, double[] t,
                                      double maxError, int maxSeg) {
        double[][] ctrl = fitSingle(xs, ys, t);
        Curve segCurve = buildCurveFromCtrl(ctrl);

        // 误差检测
        double maxErr = 0;
        int splitIdx = -1;
        for (int i = 0; i < xs.length; i++) {
            Pair p = Bezier2D.eval(segCurve, t[i]);
            double err = Math.hypot(p.getX() - xs[i], p.getY() - ys[i]);
            if (err > maxErr) {
                maxErr = err;
                splitIdx = i;
            }
        }

        if (maxErr <= maxError || maxSeg <= 1 || xs.length < 4)
            return segCurve;

        if (splitIdx <= 0) splitIdx = 1;
        if (splitIdx >= xs.length - 1) splitIdx = xs.length - 2;

        // 左右子集
        double[] leftX = Arrays.copyOfRange(xs, 0, splitIdx + 1);
        double[] leftY = Arrays.copyOfRange(ys, 0, splitIdx + 1);
        double[] leftT = new ChordLengthTable(leftX, leftY).getParameters();

        double[] rightX = Arrays.copyOfRange(xs, splitIdx, xs.length);
        double[] rightY = Arrays.copyOfRange(ys, splitIdx, ys.length);
        double[] rightT = new ChordLengthTable(rightX, rightY).getParameters();

        Curve left = fitRecursive(leftX, leftY, leftT, maxError, maxSeg / 2);
        Curve right = fitRecursive(rightX, rightY, rightT, maxError, maxSeg / 2);
        return Bezier2D.join(left, right);
    }

    /**
     * 单段最小二乘（固定首末点）。
     * @return 控制点数组 {{x0,y0},{x1,y1},{x2,y2},{x3,y3}}
     */
    private static double[][] fitSingle(double[] xs, double[] ys, double[] t) {
        int n = xs.length;
        double x0 = xs[0], y0 = ys[0];
        double x3 = xs[n-1], y3 = ys[n-1];

        double c00 = 0, c01 = 0, c11 = 0;
        double bx = 0, by = 0, dx = 0, dy = 0;
        for (int i = 0; i < n; i++) {
            double ti = t[i], mt = 1 - ti;
            double a1 = 3 * mt * mt * ti;
            double a2 = 3 * mt * ti * ti;
            double qx = xs[i] - (mt*mt*mt*x0) - (ti*ti*ti*x3);
            double qy = ys[i] - (mt*mt*mt*y0) - (ti*ti*ti*y3);
            c00 += a1 * a1;
            c01 += a1 * a2;
            c11 += a2 * a2;
            bx += a1 * qx; by += a1 * qy;
            dx += a2 * qx; dy += a2 * qy;
        }
        double det = c00 * c11 - c01 * c01;
        double p1x, p1y, p2x, p2y;
        if (Math.abs(det) < 1e-12) {
            p1x = x0 + (x3 - x0) / 3;
            p1y = y0 + (y3 - y0) / 3;
            p2x = x0 + 2 * (x3 - x0) / 3;
            p2y = y0 + 2 * (y3 - y0) / 3;
        } else {
            p1x = (c11*bx - c01*dx) / det;
            p2x = (-c01*bx + c00*dx) / det;
            p1y = (c11*by - c01*dy) / det;
            p2y = (-c01*by + c00*dy) / det;
        }
        return new double[][]{{x0,y0},{p1x,p1y},{p2x,p2y},{x3,y3}};
    }

    /** 从控制点数组构建单段 Curve（锚点+手柄） */
    private static Curve buildCurveFromCtrl(double[][] ctrl) {
        ControlPoint start = new ControlPoint()
                .setX(ctrl[0][0]).setY(ctrl[0][1])
                .setDx2(ctrl[1][0] - ctrl[0][0])
                .setDy2(ctrl[1][1] - ctrl[0][1]);
        ControlPoint end = new ControlPoint()
                .setX(ctrl[3][0]).setY(ctrl[3][1])
                .setDx1(ctrl[2][0] - ctrl[3][0])
                .setDy1(ctrl[2][1] - ctrl[3][1]);
        return new Curve(Arrays.asList(start, end), false);
    }

}