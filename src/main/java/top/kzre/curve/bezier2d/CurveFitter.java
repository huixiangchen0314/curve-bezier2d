package top.kzre.curve.bezier2d;

import java.util.Arrays;

public final class CurveFitter {
    private CurveFitter() {}

    public static Curve fit(double[] xs, double[] ys, double maxError, int maxSeg) {
        if (xs == null || ys == null || xs.length != ys.length || xs.length < 2)
            throw new IllegalArgumentException("Invalid input arrays");

        double[] t;
        if (xs.length >= 4) {
            t = CentripetalParams.centripetal(xs, ys);
        } else {
            t = new ChordLengthTable(xs, ys).getParameters();
        }
        return fitRecursive(xs, ys, t, maxError, maxSeg);
    }

    private static Curve fitRecursive(double[] xs, double[] ys, double[] t,
                                      double maxError, int maxSeg) {
        // 三点输入的特殊处理
        if (xs.length == 3) {
            // 尝试直接拟合
            double[][] ctrl = fitSingle(xs, ys, t);
            if (!isValidControlPoints(ctrl)) {
                ctrl = generateSafeLineCtrl(xs, ys);
            }
            Curve curve = buildCurveFromCtrl(ctrl);
            double[] refinedT = refineParameters(xs, ys, curve, t);
            ctrl = fitSingle(xs, ys, refinedT);
            if (!isValidControlPoints(ctrl)) {
                ctrl = generateSafeLineCtrl(xs, ys);
            }
            curve = buildCurveFromCtrl(ctrl);
            double maxErr = computeMaxError(xs, ys, curve, refinedT);
            if (maxErr <= maxError) return curve;

            // 误差仍大，在中点分割
            double[] leftX = {xs[0], xs[1]};
            double[] leftY = {ys[0], ys[1]};
            double[] leftT = new ChordLengthTable(leftX, leftY).getParameters();
            Curve left = fitRecursive(leftX, leftY, leftT, maxError, maxSeg / 2);

            double[] rightX = {xs[1], xs[2]};
            double[] rightY = {ys[1], ys[2]};
            double[] rightT = new ChordLengthTable(rightX, rightY).getParameters();
            Curve right = fitRecursive(rightX, rightY, rightT, maxError, maxSeg / 2);
            return Bezier2D.join(left, right);
        }

        // 正常点数 >=4 的处理
        double[][] ctrl = fitSingle(xs, ys, t);
        if (!isValidControlPoints(ctrl)) {
            ctrl = generateSafeLineCtrl(xs, ys);
        }
        Curve segCurve = buildCurveFromCtrl(ctrl);
        double[] refinedT = refineParameters(xs, ys, segCurve, t);
        ctrl = fitSingle(xs, ys, refinedT);
        if (!isValidControlPoints(ctrl)) {
            ctrl = generateSafeLineCtrl(xs, ys);
        }
        segCurve = buildCurveFromCtrl(ctrl);

        // 误差检测
        double maxErr = computeMaxError(xs, ys, segCurve, refinedT);
        if (maxErr <= maxError || maxSeg <= 1 || xs.length < 4)
            return segCurve;

        // 分割
        int splitIdx = 0;
        double maxLocalErr = 0;
        for (int i = 0; i < xs.length; i++) {
            Pair p = Bezier2D.eval(segCurve, refinedT[i]);
            double err = Math.hypot(p.getX() - xs[i], p.getY() - ys[i]);
            if (err > maxLocalErr) {
                maxLocalErr = err;
                splitIdx = i;
            }
        }
        splitIdx = Math.max(1, Math.min(splitIdx, xs.length - 2));

        double[] leftX = Arrays.copyOfRange(xs, 0, splitIdx + 1);
        double[] leftY = Arrays.copyOfRange(ys, 0, splitIdx + 1);
        double[] leftT = new ChordLengthTable(leftX, leftY).getParameters();
        Curve left = fitRecursive(leftX, leftY, leftT, maxError, maxSeg / 2);

        double[] rightX = Arrays.copyOfRange(xs, splitIdx, xs.length);
        double[] rightY = Arrays.copyOfRange(ys, splitIdx, ys.length);
        double[] rightT = new ChordLengthTable(rightX, rightY).getParameters();
        Curve right = fitRecursive(rightX, rightY, rightT, maxError, maxSeg / 2);

        return Bezier2D.join(left, right);
    }

    // 参数校正：牛顿法迭代 3 次
    private static double[] refineParameters(double[] xs, double[] ys, Curve curve, double[] initT) {
        double[] t = initT.clone();
        for (int iter = 0; iter < 3; iter++) {
            for (int i = 1; i < t.length - 1; i++) {
                Pair p = Bezier2D.eval(curve, t[i]);
                Pair d1 = Bezier2D.deriv(curve, t[i]);
                double dx = p.getX() - xs[i];
                double dy = p.getY() - ys[i];
                double dot = dx * d1.getX() + dy * d1.getY();
                double denom = d1.getX() * d1.getX() + d1.getY() * d1.getY() + 1e-12;
                double dt = dot / denom;
                t[i] = Math.max(0.0, Math.min(1.0, t[i] - dt));
            }
        }
        return t;
    }

    private static double computeMaxError(double[] xs, double[] ys, Curve curve, double[] t) {
        double maxErr = 0;
        for (int i = 0; i < xs.length; i++) {
            Pair p = Bezier2D.eval(curve, t[i]);
            double err = Math.hypot(p.getX() - xs[i], p.getY() - ys[i]);
            if (err > maxErr) maxErr = err;
        }
        return maxErr;
    }

    private static boolean isValidControlPoints(double[][] ctrl) {
        return Double.isFinite(ctrl[1][0]) && Double.isFinite(ctrl[1][1])
                && Double.isFinite(ctrl[2][0]) && Double.isFinite(ctrl[2][1]);
    }

    private static double[][] generateSafeLineCtrl(double[] xs, double[] ys) {
        double x0 = xs[0], y0 = ys[0];
        double x3 = xs[xs.length-1], y3 = ys[ys.length-1];
        return new double[][]{
                {x0, y0},
                {x0 + (x3 - x0) / 3, y0 + (y3 - y0) / 3},
                {x0 + 2*(x3 - x0) / 3, y0 + 2*(y3 - y0) / 3},
                {x3, y3}
        };
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