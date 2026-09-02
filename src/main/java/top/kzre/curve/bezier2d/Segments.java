package top.kzre.curve.bezier2d;

public final class Segments {
    private Segments() {}

    public static Pair eval(Segment seg, double t) {
        double mt = 1 - t;
        double mt2 = mt * mt, t2 = t * t;
        double x = mt2*mt*seg.getA().getX() + 3*mt2*t*seg.getB().getX()
                + 3*mt*t2*seg.getC().getX() + t2*t*seg.getD().getX();
        double y = mt2*mt*seg.getA().getY() + 3*mt2*t*seg.getB().getY()
                + 3*mt*t2*seg.getC().getY() + t2*t*seg.getD().getY();
        return new Pair(x, y);
    }


    /**
     * 基于四个控制点调整贝塞尔段，使其在参数 t 处经过目标点 (x, y)。
     * 保持端点 P0 和 P3 固定，调整控制点 P1 和 P2 沿原方向缩放。
     * 返回新段。
     */
    public static Segment fit(Pair p0, Pair p1, Pair p2, Pair p3, double t, double x, double y) {
        // 若 t 在端点附近，直接返回原段（端点固定无法调整）
        if (t < 1e-6 || t > 1 - 1e-6) {
            return new Segment(p0, p1, p2, p3);
        }

        // 计算原手柄向量
        double d1x = p1.getX() - p0.getX();
        double d1y = p1.getY() - p0.getY();
        double d2x = p2.getX() - p3.getX();
        double d2y = p2.getY() - p3.getY();

        // 计算 A = B(t) 在 u=0, v=0 时的位置
        double mt = 1 - t;
        double mt2 = mt * mt;
        double t2 = t * t;
        double A_x = mt2 * mt * p0.getX() + 3 * mt2 * t * p0.getX()
                + 3 * mt * t2 * p3.getX() + t2 * t * p3.getX();
        double A_y = mt2 * mt * p0.getY() + 3 * mt2 * t * p0.getY()
                + 3 * mt * t2 * p3.getY() + t2 * t * p3.getY();

        // 计算 B1 = 3*(1-t)^2*t * D1
        double B1x = 3 * mt2 * t * d1x;
        double B1y = 3 * mt2 * t * d1y;
        // 计算 B2 = 3*(1-t)*t^2 * D2
        double B2x = 3 * mt * t2 * d2x;
        double B2y = 3 * mt * t2 * d2y;

        // 目标偏移量 R = Q - A
        double Rx = x - A_x;
        double Ry = y - A_y;

        // 求解线性方程组：B1*u + B2*v = R
        double det = B1x * B2y - B1y * B2x;
        double u, v;
        if (Math.abs(det) > 1e-12) {
            // 克莱姆法则
            u = (Rx * B2y - Ry * B2x) / det;
            v = (B1x * Ry - B1y * Rx) / det;
        } else {
            // 奇异情况：取最小二乘解
            double normB1 = B1x * B1x + B1y * B1y;
            double normB2 = B2x * B2x + B2y * B2y;
            if (normB1 > normB2) {
                u = (Rx * B1x + Ry * B1y) / normB1;
                v = 0;
            } else if (normB2 > 1e-12) {
                u = 0;
                v = (Rx * B2x + Ry * B2y) / normB2;
            } else {
                // 无解，返回原段
                return new Segment(p0, p1, p2, p3);
            }
        }

        // 限制 u, v 在合理范围，避免控制点过度移动（例如 0.1 到 10）
        double MIN_SCALE = 0.1;
        double MAX_SCALE = 10.0;
        u = Math.max(MIN_SCALE, Math.min(u, MAX_SCALE));
        v = Math.max(MIN_SCALE, Math.min(v, MAX_SCALE));

        // 构造新的控制点
        Pair newP1 = new Pair(p0.getX() + u * d1x, p0.getY() + u * d1y);
        Pair newP2 = new Pair(p3.getX() + v * d2x, p3.getY() + v * d2y);

        return new Segment(p0, newP1, newP2, p3);
    }

    /**
     * 调整贝塞尔段，使其在参数 t 处经过目标点 (x, y)。
     * 保持端点 P0 和 P3 固定，调整控制点 P1 和 P2 沿原方向缩放。
     * 返回新段。
     */
    public static Segment fit(Segment seg, double t, double x, double y) {
        Pair p0 = seg.getA();
        Pair p1 = seg.getB();
        Pair p2 = seg.getC();
        Pair p3 = seg.getD();
        return fit(p0, p1, p2, p3, t, x, y);
    }

    /**
     * 基于四个 ControlPoint 调整贝塞尔段，使其在参数 t 处经过目标点 (x, y)。
     * 保持端点 P0 和 P3 固定，调整控制点 P1 和 P2 沿原方向缩放。
     * 返回新段。
     */
    public static Segment fit(ControlPoint p0, ControlPoint p1, ControlPoint p2, ControlPoint p3,
                              double t, double x, double y) {
        Pair pair0 = new Pair(p0.getX(), p0.getY());
        Pair pair1 = new Pair(p1.getX(), p1.getY());
        Pair pair2 = new Pair(p2.getX(), p2.getY());
        Pair pair3 = new Pair(p3.getX(), p3.getY());
        return fit(pair0, pair1, pair2, pair3, t, x, y);
    }

    public static Pair deriv(Segment seg, double t) {
        double mt = 1 - t;
        double dx = 3*(mt*mt*(seg.getB().getX()-seg.getA().getX())
                + 2*mt*t*(seg.getC().getX()-seg.getB().getX())
                + t*t*(seg.getD().getX()-seg.getC().getX()));
        double dy = 3*(mt*mt*(seg.getB().getY()-seg.getA().getY())
                + 2*mt*t*(seg.getC().getY()-seg.getB().getY())
                + t*t*(seg.getD().getY()-seg.getC().getY()));
        return new Pair(dx, dy);
    }

    public static Pair deriv2(Segment seg, double t) {
        double mx = seg.getA().getX() - 2*seg.getB().getX() + seg.getC().getX();
        double my = seg.getA().getY() - 2*seg.getB().getY() + seg.getC().getY();
        double nx = seg.getB().getX() - 2*seg.getC().getX() + seg.getD().getX();
        double ny = seg.getB().getY() - 2*seg.getC().getY() + seg.getD().getY();
        double dx = 6*((1-t)*mx + t*nx);
        double dy = 6*((1-t)*my + t*ny);
        return new Pair(dx, dy);
    }

    public static void split(Segment seg, double t, Segment left, Segment right) {
        double x0=seg.getA().getX(), y0=seg.getA().getY();
        double x1=seg.getB().getX(), y1=seg.getB().getY();
        double x2=seg.getC().getX(), y2=seg.getC().getY();
        double x3=seg.getD().getX(), y3=seg.getD().getY();
        double mt = 1-t;
        double x01=mt*x0+t*x1, y01=mt*y0+t*y1;
        double x12=mt*x1+t*x2, y12=mt*y1+t*y2;
        double x23=mt*x2+t*x3, y23=mt*y2+t*y3;
        double x012=mt*x01+t*x12, y012=mt*y01+t*y12;
        double x123=mt*x12+t*x23, y123=mt*y12+t*y23;
        double x0123=mt*x012+t*x123, y0123=mt*y012+t*y123;
        left.set(new Pair(x0,y0), new Pair(x01,y01), new Pair(x012,y012), new Pair(x0123,y0123));
        right.set(new Pair(x0123,y0123), new Pair(x123,y123), new Pair(x23,y23), new Pair(x3,y3));
    }

    public static AABB aabb(Segment seg) {
        double[] xs = {seg.getA().getX(), seg.getB().getX(), seg.getC().getX(), seg.getD().getX()};
        double[] ys = {seg.getA().getY(), seg.getB().getY(), seg.getC().getY(), seg.getD().getY()};

        // 计算贝塞尔曲线 X(t) 和 Y(t) 的导数系数
        // X(t) = ax*t^3 + bx*t^2 + cx*t + dx
        // X'(t) = 3*ax*t^2 + 2*bx*t + cx
        double ax = -xs[0] + 3*xs[1] - 3*xs[2] + xs[3];
        double bx = 3*xs[0] - 6*xs[1] + 3*xs[2];
        double cx = -3*xs[0] + 3*xs[1];
        double ay = -ys[0] + 3*ys[1] - 3*ys[2] + ys[3];
        double by = 3*ys[0] - 6*ys[1] + 3*ys[2];
        double cy = -3*ys[0] + 3*ys[1];

        // 收集候选 t 值：端点 t=0, t=1，以及 X 和 Y 方向的极值点
        double[] tValues = new double[6]; // 最多 0,1 + 4个极值点
        int count = 0;
        tValues[count++] = 0;
        tValues[count++] = 1;

        // 求 X 方向导数零点
        double[] roots = new double[2];
        int numX = solveQuadratic(3*ax, 2*bx, cx, roots);
        for (int i = 0; i < numX; i++) {
            double t = roots[i];
            if (t > 0 && t < 1) {
                tValues[count++] = t;
            }
        }
        // 求 Y 方向导数零点
        int numY = solveQuadratic(3*ay, 2*by, cy, roots);
        for (int i = 0; i < numY; i++) {
            double t = roots[i];
            if (t > 0 && t < 1) {
                tValues[count++] = t;
            }
        }

        double minX = Double.POSITIVE_INFINITY, minY = Double.POSITIVE_INFINITY;
        double maxX = Double.NEGATIVE_INFINITY, maxY = Double.NEGATIVE_INFINITY;
        for (int i = 0; i < count; i++) {
            double t = tValues[i];
            Pair p = eval(seg, t);
            double x = p.getX(), y = p.getY();
            if (x < minX) minX = x;
            if (y < minY) minY = y;
            if (x > maxX) maxX = x;
            if (y > maxY) maxY = y;
        }
        return new AABB(minX, minY, maxX, maxY);
    }

    /**
     * 求解二次方程 a*t^2 + b*t + c = 0 在 (0,1) 内的根。
     * 将根存入 roots 数组，返回找到的根的个数。
     */
    private static int solveQuadratic(double a, double b, double c, double[] roots) {
        if (Math.abs(a) < 1e-12) { // 退化为线性方程 b*t + c = 0
            if (Math.abs(b) < 1e-12) return 0;
            double t = -c / b;
            if (t > 0 && t < 1) {
                roots[0] = t;
                return 1;
            }
            return 0;
        }
        double disc = b*b - 4*a*c;
        if (disc < 0) return 0;
        double sqrtDisc = Math.sqrt(disc);
        double t1 = (-b - sqrtDisc) / (2*a);
        double t2 = (-b + sqrtDisc) / (2*a);
        int n = 0;
        if (t1 > 0 && t1 < 1) roots[n++] = t1;
        if (t2 > 0 && t2 < 1) roots[n++] = t2;
        return n;
    }

    /**
     * 计算贝塞尔段上参数 t 处的曲率。
     * 曲率公式：κ = |P' × P''| / |P'|^3
     */
    public static double curvature(Segment seg, double t) {
        Pair d1 = deriv(seg, t);
        Pair d2 = deriv2(seg, t);
        double cross = d1.getX() * d2.getY() - d1.getY() * d2.getX();
        double len = Math.hypot(d1.getX(), d1.getY());
        if (len < 1e-12) {
            return 0.0;
        }
        return cross / (len * len * len);
    }

    public static Segment translate(Segment seg, double dx, double dy) {
        return new Segment(
                new Pair(seg.getA().getX()+dx, seg.getA().getY()+dy),
                new Pair(seg.getB().getX()+dx, seg.getB().getY()+dy),
                new Pair(seg.getC().getX()+dx, seg.getC().getY()+dy),
                new Pair(seg.getD().getX()+dx, seg.getD().getY()+dy));
    }

    public static Segment scale(Segment seg, double sx, double sy, double cx, double cy) {
        return new Segment(
                scalePoint(seg.getA(),sx,sy,cx,cy),
                scalePoint(seg.getB(),sx,sy,cx,cy),
                scalePoint(seg.getC(),sx,sy,cx,cy),
                scalePoint(seg.getD(),sx,sy,cx,cy));
    }
    private static Pair scalePoint(Pair p, double sx, double sy, double cx, double cy) {
        return new Pair(cx+(p.getX()-cx)*sx, cy+(p.getY()-cy)*sy);
    }

    /**
     * 测试段是否足够平坦，可以用直线段近似。
     * flatnessSq 是允许的最大距离平方（通常设为 (0.25像素)^2 或类似值）。
     */
    public static boolean isFlat(Segment seg, double flatnessSq) {
        Pair A = seg.getA();
        Pair D = seg.getD();
        double dx = D.getX() - A.getX();
        double dy = D.getY() - A.getY();
        double lenSq = dx * dx + dy * dy;

        // 如果端点重合，则检查控制点是否也都重合
        if (lenSq < 1e-12) {
            return pointDistSq(seg.getB(), A) <= flatnessSq
                    && pointDistSq(seg.getC(), A) <= flatnessSq;
        }

        // 控制点 B 和 C 到直线 AD 的垂直距离平方
        if (pointToLineDistSq(seg.getB(), A, dx, dy, lenSq) > flatnessSq) return false;
        if (pointToLineDistSq(seg.getC(), A, dx, dy, lenSq) > flatnessSq) return false;

        return true;
    }

    /** 点到点的欧氏距离平方 */
    private static double pointDistSq(Pair p, Pair q) {
        double dx = p.getX() - q.getX();
        double dy = p.getY() - q.getY();
        return dx * dx + dy * dy;
    }

    /** 点到直线（由 A 和方向向量 dx,dy 定义）的垂直距离平方 */
    private static double pointToLineDistSq(Pair p, Pair A,
                                            double dx, double dy, double lenSq) {
        double cross = (p.getX() - A.getX()) * dy - (p.getY() - A.getY()) * dx;
        return (cross * cross) / lenSq;
    }
}