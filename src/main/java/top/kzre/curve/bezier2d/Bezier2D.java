package top.kzre.curve.bezier2d;

public final class Bezier2D {
    private static final Spec impl = new Bezier2DImpl();

    private Bezier2D() {}

    public static Pair eval(Curve curve, double t) {
        return impl.eval(curve, t);
    }

    public static Pair deriv(Curve curve, double t) {
        return impl.deriv(curve, t);
    }

    public static Pair deriv2(Curve curve, double t) {
        return impl.deriv2(curve, t);
    }

    public static void split(Curve curve, double t, Curve out1, Curve out2) {
        impl.split(curve, t, out1, out2);
    }

    public static AABB aabb(Curve curve) {
        return impl.aabb(curve);
    }

    public static Curve translate(Curve curve, double dx, double dy) {
        return impl.translate(curve, dx, dy);
    }

    public static Curve scale(Curve curve, double sx, double sy, double cx, double cy) {
        return impl.scale(curve, sx, sy, cx, cy);
    }

    public static Curve fit(double[] xs, double[] ys, double maxError, int maxSeg) {
        return impl.fit(xs, ys, maxError, maxSeg);
    }

    public static Curve join(Curve left, Curve right) {
        return impl.join(left, right);
    }

    public static void insertPoint(Curve curve, double t) {
        impl.insertPoint(curve, t);
    }
    public static void deletePoint(Curve curve, int idx) {
        impl.deletePoint(curve, idx);
    }

    public static void reform(Curve curve, int count) {
        impl.reform(curve, count);
    }

    public static ClosestPointResult closestPoint(Curve curve, Pair point) {
        return impl.closestPoint(curve, point);
    }

    public static Curve reverse(Curve curve) {
        return impl.reverse(curve);
    }
    public static Pair unitTangent(Curve curve, double t) {
        return impl.unitTangent(curve, t);
    }
    public static Pair unitNormal(Curve curve, double t) {
        return impl.unitNormal(curve, t);
    }
    public static Curve offset(Curve curve, double distance) {
        return impl.offset(curve, distance);
    }
    public static double curvature(Curve curve, double t) {
        return impl.curvature(curve, t);
    }
    public static Pair[] sample(Curve curve, int count) {
        return impl.sample(curve, count);
    }

    public static void divide(Curve curve, int idx, Curve out1, Curve out2) {
        impl.divide(curve, idx, out1, out2);
    }

    public interface Spec {
        /**
         * 求值
         */
         Pair eval(Curve curve, double t);

        /**
         * 一阶导
         */
         Pair deriv(Curve curve, double t);
        /**
         * 二阶导
         */
         Pair deriv2(Curve curve, double t);
         void divide(Curve curve, int idx, Curve out1, Curve out2);

         void split(Curve curve, double t, Curve out1, Curve out2);

         AABB aabb(Curve curve);

         Curve translate(Curve curve, double dx, double dy);

         Curve scale(Curve curve, double sx, double sy, double cx, double cy);

         Curve fit(double[] xs, double[] ys, double maxError, int maxSeg);

         Curve join(Curve left, Curve right);

         void insertPoint(Curve curve, double t);

         void deletePoint(Curve curve, int idx);
         void reform(Curve curve, int count);

         ClosestPointResult closestPoint(Curve curve, Pair point);

        /** 曲线反向，返回新 Curve */
        Curve reverse(Curve curve);

        /** 单位切向量 */
        Pair unitTangent(Curve curve, double t);

        /** 单位法向量（左手系，指向左侧） */
        Pair unitNormal(Curve curve, double t);

        /** 曲线偏移（近似），返回新 Curve */
        Curve offset(Curve curve, double distance);

        /** 曲率 */
        double curvature(Curve curve, double t);

        /** 均匀采样曲线，返回 count 个点（首尾包含） */
        Pair[] sample(Curve curve, int count);

    }
}
