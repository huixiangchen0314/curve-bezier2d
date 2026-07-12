package top.kzre.curve.bezier2d;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.Arrays;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class Bezier2DImplTest {

    private Bezier2DImpl impl;
    private Curve curve;          // 拱形三次贝塞尔： (0,0)->(10,20)->(30,20)->(40,0)
    private Curve lineCurve;      // 直线： (0,0)->(40,0)

    @BeforeEach
    void setUp() {
        impl = new Bezier2DImpl();

        // 创建拱形曲线（两个锚点，带手柄）—— 使用 Arrays.asList 兼容 Java 8
        ControlPoint start = new ControlPoint()
                .setX(0).setY(0)
                .setDx2(10).setDy2(20);
        ControlPoint end = new ControlPoint()
                .setX(40).setY(0)
                .setDx1(-10).setDy1(20);
        curve = new Curve(Arrays.asList(start, end), false);

        // 创建直线曲线
        ControlPoint ls = new ControlPoint().setX(0).setY(0);
        ControlPoint le = new ControlPoint().setX(40).setY(0);
        lineCurve = new Curve(Arrays.asList(ls, le), false);
    }

    @Test
    void eval() {
        Pair p0 = impl.eval(curve, 0.0);
        assertEquals(0.0, p0.getX(), 1e-9);
        assertEquals(0.0, p0.getY(), 1e-9);

        Pair pMid = impl.eval(curve, 0.5);
        // t=0.5 时，三次贝塞尔的中点公式： (P0+3P1+3P2+P3)/8
        double expectedX = (0 + 3*10 + 3*30 + 40) / 8.0;  // (0+30+90+40)/8 = 160/8=20
        double expectedY = (0 + 3*20 + 3*20 + 0) / 8.0;   // (0+60+60+0)/8 = 120/8=15
        assertEquals(expectedX, pMid.getX(), 1e-9);
        assertEquals(expectedY, pMid.getY(), 1e-9);

        Pair p1 = impl.eval(curve, 1.0);
        assertEquals(40.0, p1.getX(), 1e-9);
        assertEquals(0.0, p1.getY(), 1e-9);
    }

    @Test
    void deriv() {
        Pair d0 = impl.deriv(curve, 0.0);
        // 导数 = 3*(P1-P0) = 3*(10,20) = (30,60)
        assertEquals(30.0, d0.getX(), 1e-9);
        assertEquals(60.0, d0.getY(), 1e-9);

        Pair d1 = impl.deriv(curve, 1.0);
        // 导数 = 3*(P3-P2) = 3*(40-30, 0-20) = 3*(10,-20) = (30,-60)
        assertEquals(30.0, d1.getX(), 1e-9);
        assertEquals(-60.0, d1.getY(), 1e-9);
    }

    @Test
    void deriv2() {
        Pair dd = impl.deriv2(curve, 0.5);
        // 二阶导公式复杂，至少验证不是零（因为不是直线）
        assertNotEquals(0.0, dd.getX() + dd.getY(), 1e-6);
        // 直线二阶导应为0
        Pair ddLine = impl.deriv2(lineCurve, 0.5);
        assertEquals(0.0, ddLine.getX(), 1e-9);
        assertEquals(0.0, ddLine.getY(), 1e-9);
    }

    @Test
    void aabb() {
        AABB box = impl.aabb(curve);
        // 控制点包围盒：minX=0, minY=0, maxX=40, maxY=20
        assertEquals(0.0, box.getMinX());
        assertEquals(0.0, box.getMinY());
        assertEquals(40.0, box.getMaxX());
        assertEquals(20.0, box.getMaxY());
    }

    @Test
    void translate() {
        Curve moved = impl.translate(curve, 10, -5);
        Pair p0 = impl.eval(moved, 0.0);
        assertEquals(10.0, p0.getX(), 1e-9);
        assertEquals(-5.0, p0.getY(), 1e-9);
        Pair p1 = impl.eval(moved, 1.0);
        assertEquals(50.0, p1.getX(), 1e-9);
        assertEquals(-5.0, p1.getY(), 1e-9);
        // 手柄也应被平移（通过 ControlPoint 的 dx2 等平移检查）
        ControlPoint start = moved.getPoints().get(0);
        assertEquals(10.0, start.getDx2(), 1e-9);  // 平移不改变手柄向量
        assertEquals(20.0, start.getDy2(), 1e-9);
    }

    @Test
    void scale() {
        Curve scaled = impl.scale(curve, 2, 2, 20, 0); // 以 (20,0) 为中心放大2倍
        Pair p0 = impl.eval(scaled, 0.0);
        // 原点 (0,0) 缩放后： (20 + (0-20)*2 = -20, 0)
        assertEquals(-20.0, p0.getX(), 1e-9);
        assertEquals(0.0, p0.getY(), 1e-9);
        // 手柄也应该按比例缩放
        ControlPoint start = scaled.getPoints().get(0);
        assertEquals(20.0, start.getDx2(), 1e-9);  // dx2: 10*2=20
        assertEquals(40.0, start.getDy2(), 1e-9);  // dy2: 20*2=40
    }

    @Test
    void divide() {
        // 创建一个4个控制点的曲线
        List<ControlPoint> pts = Arrays.asList(
                new ControlPoint().setX(0).setY(0),
                new ControlPoint().setX(10).setY(20),
                new ControlPoint().setX(30).setY(20),
                new ControlPoint().setX(40).setY(0));
        Curve multi = new Curve(pts, false);
        Curve left = new Curve(Arrays.asList(new ControlPoint(), new ControlPoint()), false);
        Curve right = new Curve(Arrays.asList(new ControlPoint(), new ControlPoint()), false);
        impl.divide(multi, 2, left, right);
        // left 应有 3 个锚点 (0,1,2)
        assertEquals(3, left.getPoints().size());
        // right 应有 2 个锚点 (2,3)
        assertEquals(2, right.getPoints().size());
        // 验证左右曲线在分割点连续
        Pair leftEnd = impl.eval(left, 1.0);
        Pair rightStart = impl.eval(right, 0.0);
        assertEquals(leftEnd.getX(), rightStart.getX(), 1e-9);
        assertEquals(leftEnd.getY(), rightStart.getY(), 1e-9);
    }

    @Test
    void split() {
        Curve left = new Curve(Arrays.asList(
                new ControlPoint(), new ControlPoint()), false);
        Curve right = new Curve(Arrays.asList(
                new ControlPoint(), new ControlPoint()), false);
        double splitT = 0.5;
        impl.split(curve, splitT, left, right);

        // 左段局部参数 u ∈ [0,1] 对应原曲线 t = u * splitT
        for (int i = 0; i <= 10; i++) {
            double u = i / 10.0;
            double origT = u * splitT;
            Pair expected = impl.eval(curve, origT);
            Pair actual   = impl.eval(left, u);
            assertEquals(expected.getX(), actual.getX(), 1e-6);
            assertEquals(expected.getY(), actual.getY(), 1e-6);
        }

        // 右段局部参数 u ∈ [0,1] 对应原曲线 t = splitT + u * (1 - splitT)
        for (int i = 0; i <= 10; i++) {
            double u = i / 10.0;
            double origT = splitT + u * (1 - splitT);
            Pair expected = impl.eval(curve, origT);
            Pair actual   = impl.eval(right, u);
            assertEquals(expected.getX(), actual.getX(), 1e-6);
            assertEquals(expected.getY(), actual.getY(), 1e-6);
        }
    }

    @Test
    void fit() {
        // 用拱形曲线的采样点作为输入，拟合应接近原曲线
        double[] xs = {0, 10, 20, 30, 40};
        double[] ys = {0, 20, 100, 20, 0};
        Curve fitted = impl.fit(xs, ys, 0.1, 10);
        assertNotNull(fitted);
        // 至少能拟合出控制点，且端点接近
        Pair start = impl.eval(fitted, 0.0);
        assertEquals(0.0, start.getX(), 1e-2);
        assertEquals(0.0, start.getY(), 1e-2);
        Pair end = impl.eval(fitted, 1.0);
        assertEquals(40.0, end.getX(), 1e-2);
        assertEquals(0.0, end.getY(), 1e-2);
    }

    @Test
    void join() {
        // 两条曲线首尾连接
        Curve c1 = new Curve(Arrays.asList(
                new ControlPoint().setX(0).setY(0),
                new ControlPoint().setX(20).setY(10)), false);
        Curve c2 = new Curve(Arrays.asList(
                new ControlPoint().setX(20).setY(10),
                new ControlPoint().setX(40).setY(0)), false);
        Curve joined = impl.join(c1, c2);
        // 连接后的曲线在连接点处应 G1 连续，导数方向相反？
        Pair tan1 = impl.deriv(joined, 0.5); // 简单假设连接点在 t=0.5（实际可能不是均匀，但此处仅检查不崩溃）
        assertNotNull(tan1);
        // 基本验证形状不变：起终点
        Pair p0 = impl.eval(joined, 0.0);
        assertEquals(0.0, p0.getX(), 1e-9);
        assertEquals(0.0, p0.getY(), 1e-9);
        Pair p1 = impl.eval(joined, 1.0);
        assertEquals(40.0, p1.getX(), 1e-9);
        assertEquals(0.0, p1.getY(), 1e-9);
    }

    @Test
    void insertPoint() {
        Curve copy = new Curve(
                Arrays.asList(curve.getPoints().get(0).copy(), curve.getPoints().get(1).copy()),
                false);
        // 原曲线三个关键点
        Pair origStart = impl.eval(curve, 0.0);
        Pair origMid   = impl.eval(curve, 0.5);
        Pair origEnd   = impl.eval(curve, 1.0);

        impl.insertPoint(copy, 0.5);
        assertEquals(3, copy.getPoints().size());

        // 用 closestPoint 验证形状未变
        assertTrue(impl.closestPoint(copy, origStart).getDistance() < 1e-6);
        assertTrue(impl.closestPoint(copy, origMid).getDistance() < 1e-6);
        assertTrue(impl.closestPoint(copy, origEnd).getDistance() < 1e-6);
    }

    @Test
    void deletePoint() {
        // 创建三段曲线（4个锚点），删除中间点
        List<ControlPoint> pts = Arrays.asList(
                new ControlPoint().setX(0).setY(0),
                new ControlPoint().setX(10).setY(20),
                new ControlPoint().setX(30).setY(20),
                new ControlPoint().setX(40).setY(0));
        Curve multi = new Curve(pts, false);
        impl.deletePoint(multi, 1); // 删除第二个点
        assertEquals(3, multi.getPoints().size());
        // 形状可能有轻微变化，但端点不变
        Pair p0 = impl.eval(multi, 0.0);
        assertEquals(0.0, p0.getX(), 1e-9);
        assertEquals(0.0, p0.getY(), 1e-9);
        Pair p1 = impl.eval(multi, 1.0);
        assertEquals(40.0, p1.getX(), 1e-9);
        assertEquals(0.0, p1.getY(), 1e-9);
    }

    @Test
    void reverse() {
        Curve rev = impl.reverse(curve);
        // 起点应为原终点
        Pair start = impl.eval(rev, 0.0);
        assertEquals(40.0, start.getX(), 1e-9);
        assertEquals(0.0, start.getY(), 1e-9);
        Pair end = impl.eval(rev, 1.0);
        assertEquals(0.0, end.getX(), 1e-9);
        assertEquals(0.0, end.getY(), 1e-9);
        // 中点应对应原曲线 0.5 的点，且 y 坐标相同（对称曲线）
        Pair mid = impl.eval(rev, 0.5);
        assertEquals(20.0, mid.getX(), 1e-9);
        assertEquals(15.0, mid.getY(), 1e-9);
    }

    @Test
    void unitTangent() {
        Pair t0 = impl.unitTangent(curve, 0.0);
        double len = Math.hypot(t0.getX(), t0.getY());
        assertEquals(1.0, len, 1e-9);
        // 方向应与导数一致
        Pair d0 = impl.deriv(curve, 0.0);
        assertTrue(t0.getX() * d0.getX() > 0 && t0.getY() * d0.getY() > 0);
    }

    @Test
    void unitNormal() {
        Pair n = impl.unitNormal(curve, 0.5);
        assertEquals(1.0, Math.hypot(n.getX(), n.getY()), 1e-9);
        // 应与单位切向量正交
        Pair t = impl.unitTangent(curve, 0.5);
        double dot = t.getX() * n.getX() + t.getY() * n.getY();
        assertEquals(0.0, dot, 1e-9);
    }

    @Test
    void curvature() {
        // 直线曲率为0
        double curvLine = impl.curvature(lineCurve, 0.5);
        assertEquals(0.0, curvLine, 1e-9);
        // 拱形曲线曲率非零
        double curv = impl.curvature(curve, 0.5);
        assertTrue(curv != 0);
    }

    @Test
    void sample() {
        Pair[] points = impl.sample(curve, 5);
        assertEquals(5, points.length);
        // 首尾点正确
        assertEquals(0.0, points[0].getX(), 1e-9);
        assertEquals(0.0, points[0].getY(), 1e-9);
        assertEquals(40.0, points[4].getX(), 1e-9);
        assertEquals(0.0, points[4].getY(), 1e-9);
        // 中点应大致在 (20,15)
        Pair mid = points[2];
        assertEquals(20.0, mid.getX(), 1e-6);
        assertEquals(15.0, mid.getY(), 1e-6);
    }

    @Test
    void offset() {
        Curve offsetCurve = impl.offset(curve, 5.0);
        assertNotNull(offsetCurve);
        // 偏移是近似算法，只做基本检查，不验证距离精度
        Pair start = impl.eval(offsetCurve, 0.0);
        Pair end = impl.eval(offsetCurve, 1.0);
        // 确保端点有变化
        assertTrue(Math.hypot(start.getX() - 0, start.getY() - 0) > 0.1);
        assertTrue(Math.hypot(end.getX() - 40, end.getY() - 0) > 0.1);
        // 可选：验证偏移方向大致正确（法线方向）
    }

    @Test
    void reform() {
        Curve morePoints = new Curve(
                Arrays.asList(curve.getPoints().get(0).copy(), curve.getPoints().get(1).copy()),
                false);
        Pair origStart = impl.eval(curve, 0.0);
        Pair origMid   = impl.eval(curve, 0.5);
        Pair origEnd   = impl.eval(curve, 1.0);

        // 增加点数 2→4，形状不变
        impl.reform(morePoints, 4);
        assertEquals(4, morePoints.getPoints().size());
        // 直接 eval 在 t=0.5 应精确等于原中点
        Pair newMid = impl.eval(morePoints, 0.5);
        assertEquals(origMid.getX(), newMid.getX(), 1e-9);
        assertEquals(origMid.getY(), newMid.getY(), 1e-9);
        // 起终点也可验证
        Pair newStart = impl.eval(morePoints, 0.0);
        Pair newEnd   = impl.eval(morePoints, 1.0);
        assertEquals(origStart.getX(), newStart.getX(), 1e-9);
        assertEquals(origStart.getY(), newStart.getY(), 1e-9);
        assertEquals(origEnd.getX(), newEnd.getX(), 1e-9);
        assertEquals(origEnd.getY(), newEnd.getY(), 1e-9);

        // 减少点数 4→2，通过拟合近似，起终点精确，中点误差允许稍大
        impl.reform(morePoints, 2);
        assertEquals(2, morePoints.getPoints().size());
        assertTrue(impl.closestPoint(morePoints, origStart).getDistance() < 1e-6);
        assertTrue(impl.closestPoint(morePoints, origEnd).getDistance() < 1e-6);
        assertTrue(impl.closestPoint(morePoints, origMid).getDistance() < 5.0);
    }

    @Test
    void closestPoint() {
        // 测试已知最近点：鼠标在 (20, 30)，最近点应在曲线顶部附近
        Pair mouse = new Pair(20, 30);
        ClosestPointResult res = impl.closestPoint(curve, mouse);
        assertTrue(res.getDistance() > 0);
        // 最近点应在曲线顶部 (20,15) 附近
        Pair pt = res.getPoint();
        assertTrue(Math.abs(pt.getX() - 20) < 5, "x too far");
        assertTrue(Math.abs(pt.getY() - 15) < 10, "y too far");
    }


    @Test
    void sampleHighDensity() {
        int count = 1001; // 奇数，确保中间点 t=0.5
        Pair[] pts = impl.sample(curve, count);
        assertEquals(count, pts.length);
        // 首尾点精确匹配
        Pair start = impl.eval(curve, 0.0);
        assertEquals(start.getX(), pts[0].getX(), 1e-9);
        assertEquals(start.getY(), pts[0].getY(), 1e-9);
        Pair end = impl.eval(curve, 1.0);
        assertEquals(end.getX(), pts[count-1].getX(), 1e-9);
        assertEquals(end.getY(), pts[count-1].getY(), 1e-9);
        // 中点（t=0.5）精确对应采样点索引 500
        Pair midExact = impl.eval(curve, 0.5);
        Pair midSampled = pts[count/2]; // 1001/2 = 500
        assertEquals(midExact.getX(), midSampled.getX(), 1e-9);
        assertEquals(midExact.getY(), midSampled.getY(), 1e-9);
    }

    @Test
    void insertPointPrecision() {
        // 插入点前后，采样1000个点的最大误差
        Curve copy = new Curve(Arrays.asList(curve.getPoints().get(0).copy(), curve.getPoints().get(1).copy()), false);
        Pair[] originalSample = impl.sample(curve, 1000);
        impl.insertPoint(copy, 0.3); // 在 0.3 处插入
        Pair[] newSample = impl.sample(copy, 1000);
        double maxErr = 0;
        for (int i = 0; i < 1000; i++) {
            double t = (double)i / 999.0;
            Pair orig = impl.eval(curve, t);
            Pair modified = impl.eval(copy, t);
            double err = Math.hypot(orig.getX() - modified.getX(), orig.getY() - modified.getY());
            maxErr = Math.max(maxErr, err);
        }
        // 由于参数化可能微调，比较 closestPoint 距离
        maxErr = 0;
        for (int i = 0; i < 1000; i++) {
            Pair orig = originalSample[i];
            double dist = impl.closestPoint(copy, orig).getDistance();
            maxErr = Math.max(maxErr, dist);
        }
        assertTrue(maxErr < 1e-6, "Max shape error after insert: " + maxErr);
    }

    @Test
    void splitPrecision() {
        // 分割后，左右段分别采样与原曲线对应区间比较
        Curve left = new Curve(Arrays.asList(new ControlPoint(), new ControlPoint()), false);
        Curve right = new Curve(Arrays.asList(new ControlPoint(), new ControlPoint()), false);
        double splitT = 0.5;
        impl.split(curve, splitT, left, right);

        // 左段均匀采样 500 点，与对应原曲线 t 比较
        double maxErr = 0;
        for (int i = 0; i <= 500; i++) {
            double u = (double)i / 500.0;
            double origT = u * splitT;
            Pair expected = impl.eval(curve, origT);
            Pair actual = impl.eval(left, u);
            double err = Math.hypot(expected.getX() - actual.getX(), expected.getY() - actual.getY());
            maxErr = Math.max(maxErr, err);
        }
        assertTrue(maxErr < 1e-6, "Left split max error: " + maxErr);

        // 右段
        maxErr = 0;
        for (int i = 0; i <= 500; i++) {
            double u = (double)i / 500.0;
            double origT = splitT + u * (1 - splitT);
            Pair expected = impl.eval(curve, origT);
            Pair actual = impl.eval(right, u);
            double err = Math.hypot(expected.getX() - actual.getX(), expected.getY() - actual.getY());
            maxErr = Math.max(maxErr, err);
        }
        assertTrue(maxErr < 1e-6, "Right split max error: " + maxErr);
    }

    @Test
    void reformPrecision() {
        // 增加点数后形状保持不变
        Curve morePoints = new Curve(Arrays.asList(curve.getPoints().get(0).copy(), curve.getPoints().get(1).copy()), false);
        impl.reform(morePoints, 10); // 增加到10个控制点
        assertEquals(10, morePoints.getPoints().size());
        Pair[] origSample = impl.sample(curve, 500);
        double maxErr = 0;
        for (Pair orig : origSample) {
            double dist = impl.closestPoint(morePoints, orig).getDistance();
            maxErr = Math.max(maxErr, dist);
        }
        assertTrue(maxErr < 1e-6, "Reform increase precision error: " + maxErr);
    }

    @Test
    void reversePrecision() {
        Curve rev = impl.reverse(curve);
        Pair[] origSample = impl.sample(curve, 500);
        double maxErr = 0;
        for (int i = 0; i < 500; i++) {
            double t = (double)i / 499.0;
            Pair orig = origSample[i];
            // 反向曲线在参数 1-t 处应找到原点
            Pair revPoint = impl.eval(rev, 1 - t);
            double err = Math.hypot(orig.getX() - revPoint.getX(), orig.getY() - revPoint.getY());
            maxErr = Math.max(maxErr, err);
        }
        assertTrue(maxErr < 1e-6, "Reverse precision error: " + maxErr);
    }

    // ── 拟合的额外测试 ───────────────────────────────
    @Test
    void testFitThreePoints() {
        double[] xs = {0, 10, 20};
        double[] ys = {0, 20, 0};
        Curve fitted = impl.fit(xs, ys, 0.1, 10);
        assertNotNull(fitted);
        assertTrue(fitted.getPoints().size() >= 2); // 至少两个锚点
        Pair start = impl.eval(fitted, 0.0);
        Pair end   = impl.eval(fitted, 1.0);
        assertEquals(0.0, start.getX(), 1e-6);
        assertEquals(0.0, start.getY(), 1e-6);
        assertEquals(20.0, end.getX(), 1e-6);
        assertEquals(0.0, end.getY(), 1e-6);
        // 中点应接近 y=10 以上 (拱起)
        Pair mid = impl.eval(fitted, 0.5);
        assertTrue(mid.getY() > 5, "middle should be raised");
    }

    @Test
    void testFitCollinearPoints() {
        double[] xs = {0, 10, 20, 30};
        double[] ys = {5, 5, 5, 5};
        Curve fitted = impl.fit(xs, ys, 0.01, 10);
        assertNotNull(fitted);
        // 共线点应拟合出近似直线，误差极小
        for (double t = 0.0; t <= 1.0; t += 0.1) {
            Pair p = impl.eval(fitted, t);
            assertEquals(5.0, p.getY(), 0.1, "Collinear Y should be near 5");
        }
    }

    @Test
    void testFitLargeAmplitude() {
        double[] xs = {0, 5, 10, 15, 20};
        double[] ys = {0, 1000, 2000, 1000, 0};
        Curve fitted = impl.fit(xs, ys, 5.0, 10); // 允许稍大误差
        assertNotNull(fitted);
        Pair start = impl.eval(fitted, 0.0);
        Pair end   = impl.eval(fitted, 1.0);
        assertEquals(0.0, start.getX(), 1e-6);
        assertEquals(0.0, start.getY(), 1e-6);
        assertEquals(20.0, end.getX(), 1e-6);
        assertEquals(0.0, end.getY(), 1e-6);
        // 中点应接近高值
        Pair mid = impl.eval(fitted, 0.5);
        assertTrue(mid.getY() > 1000, "Middle should be large");
    }

    @Test
    void testFitDensePoints() {
        int n = 100;
        double[] xs = new double[n];
        double[] ys = new double[n];
        for (int i = 0; i < n; i++) {
            double t = (double) i / (n - 1);
            xs[i] = t * 100.0;
            ys[i] = 50.0 - Math.sin(t * Math.PI) * 40.0; // 正弦波形
        }
        Curve fitted = impl.fit(xs, ys, 1.0, 20);
        assertNotNull(fitted);
        // 验证端点
        Pair start = impl.eval(fitted, 0.0);
        assertEquals(xs[0], start.getX(), 1e-6);
        assertEquals(ys[0], start.getY(), 1e-6);
        Pair end = impl.eval(fitted, 1.0);
        assertEquals(xs[n-1], end.getX(), 1e-6);
        assertEquals(ys[n-1], end.getY(), 1e-6);
        // 检查误差是否可接受（由于参数校正，误差应小于阈值）
        double maxErr = 0;
        for (int i = 0; i < n; i++) {
            double t = (double) i / (n - 1);
            Pair p = impl.eval(fitted, t);
            double err = Math.hypot(p.getX() - xs[i], p.getY() - ys[i]);
            maxErr = Math.max(maxErr, err);
        }
        // 误差不应过大（根据拟合算法，可能分段；这里放宽到10）
        assertTrue(maxErr < 10.0, "Max error too large: " + maxErr);
    }

    @Test
    void testFitFlatCurve() {
        double[] xs = {0, 1, 2, 3, 4};
        double[] ys = {0.0, 0.01, 0.02, 0.01, 0.0}; // 很平缓的拱形
        Curve fitted = impl.fit(xs, ys, 0.001, 10);
        assertNotNull(fitted);
        // 中间点的 y 应在 0.01~0.02 之间
        Pair mid = impl.eval(fitted, 0.5);
        assertTrue(mid.getY() >= 0.005 && mid.getY() <= 0.025, "Flat curve middle");
    }

    @Test
    void testFitTwoPoints() {
        double[] xs = {10, 50};
        double[] ys = {20, 80};
        Curve fitted = impl.fit(xs, ys, 0.1, 10);
        assertNotNull(fitted);
        // 两点拟合为直线
        Pair start = impl.eval(fitted, 0.0);
        Pair end   = impl.eval(fitted, 1.0);
        assertEquals(10.0, start.getX(), 1e-6);
        assertEquals(20.0, start.getY(), 1e-6);
        assertEquals(50.0, end.getX(), 1e-6);
        assertEquals(80.0, end.getY(), 1e-6);
        // 中间点应在线性插值上
        Pair mid = impl.eval(fitted, 0.5);
        assertEquals(30.0, mid.getX(), 1e-6);
        assertEquals(50.0, mid.getY(), 1e-6);
    }

    @Test
    void testFitSinglePointThrows() {
        double[] xs = {5};
        double[] ys = {10};
        assertThrows(IllegalArgumentException.class, () -> impl.fit(xs, ys, 1.0, 5));
    }

}


