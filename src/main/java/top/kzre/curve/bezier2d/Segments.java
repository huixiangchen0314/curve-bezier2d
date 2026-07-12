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
        double minX=Double.POSITIVE_INFINITY, minY=Double.POSITIVE_INFINITY;
        double maxX=Double.NEGATIVE_INFINITY, maxY=Double.NEGATIVE_INFINITY;
        Pair[] pts = {seg.getA(), seg.getB(), seg.getC(), seg.getD()};
        for (Pair p : pts) {
            if (p.getX()<minX) minX=p.getX();
            if (p.getY()<minY) minY=p.getY();
            if (p.getX()>maxX) maxX=p.getX();
            if (p.getY()>maxY) maxY=p.getY();
        }
        return new AABB(minX, minY, maxX, maxY);
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