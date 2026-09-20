package top.kzre.curve.bezier2d;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Bezier2DImpl implements Bezier2D.Spec {

    // ═══════════════════════════════════════════════════════════
    // 求值
    // ═══════════════════════════════════════════════════════════

    @Override
    public Pair eval(Curve curve, double t) {
        IndexedSegment is = curve.segmentAt(t);
        return Segments.eval(is.getSegment(), is.getLocal());
    }

    @Override
    public Pair deriv(Curve curve, double t) {
        IndexedSegment is = curve.segmentAt(t);
        return Segments.deriv(is.getSegment(), is.getLocal());
    }

    @Override
    public Pair deriv2(Curve curve, double t) {
        IndexedSegment is = curve.segmentAt(t);
        return Segments.deriv2(is.getSegment(), is.getLocal());
    }

    @Override
    public Pair tangent(Curve curve, double t) {
        Pair d = deriv(curve, t);
        double len = Math.hypot(d.getX(), d.getY());
        return len < 1e-12 ? new Pair(0, 0) : new Pair(d.getX() / len, d.getY() / len);
    }

    @Override
    public Pair normal(Curve curve, double t) {
        Pair tan = tangent(curve, t);
        return new Pair(-tan.getY(), tan.getX());
    }

    @Override
    public double curvature(Curve curve, double t) {
        Pair d1 = deriv(curve, t), d2 = deriv2(curve, t);
        double cross = d1.getX() * d2.getY() - d1.getY() * d2.getX();
        double len = Math.hypot(d1.getX(), d1.getY());
        return len < 1e-12 ? 0 : cross / (len * len * len);
    }

    @Override
    public Pair[] sample(Curve curve, int count) {
        if (count < 2) throw new IllegalArgumentException("count must be at least 2");
        Pair[] result = new Pair[count];
        for (int i = 0; i < count; i++) result[i] = eval(curve, (double) i / (count - 1));
        return result;
    }

    // ═══════════════════════════════════════════════════════════
    // 最近点
    // ═══════════════════════════════════════════════════════════

    @Override
    public ClosestPointResult closestPoint(Curve curve, Pair point) {
        int totalSegs = curve.getSegmentCount();
        if (totalSegs <= 0) {
            throw new IllegalStateException("curve has no segments");
        }
        return closestPointRange(curve, point, 0, totalSegs - 1);
    }

    @Override
    public ClosestPointResult closestPointRange(Curve curve, Pair point,
                                                int fromSeg, int toSeg) {
        int segCount = curve.getSegmentCount();
        if (fromSeg < 0 || toSeg >= segCount || fromSeg > toSeg) {
            throw new IndexOutOfBoundsException(
                    "range [" + fromSeg + ", " + toSeg + "] out of [0, " + segCount + ")");
        }
        double px = point.getX(), py = point.getY();
        double bestDist = Double.POSITIVE_INFINITY, bestGlobalT = 0;
        Pair bestPt = null;

        for (int i = fromSeg; i <= toSeg; i++) {
            Segment seg = curve.getSegment(i);
            double localBestT = 0, localBestDist = Double.POSITIVE_INFINITY;
            for (int s = 0; s <= 10; s++) {
                double t = s / 10.0;
                Pair p = Segments.eval(seg, t);
                double d = Math.hypot(p.getX() - px, p.getY() - py);
                if (d < localBestDist) { localBestDist = d; localBestT = t; }
            }
            double refinedT = refineLocalT(seg, px, py, localBestT);
            Pair refinedPt = Segments.eval(seg, refinedT);
            double refinedDist = Math.hypot(refinedPt.getX() - px, refinedPt.getY() - py);
            if (refinedDist < bestDist) {
                bestDist = refinedDist;
                bestPt = refinedPt;
                bestGlobalT = (i + refinedT) / segCount;
            }
        }
        return new ClosestPointResult(bestPt, bestGlobalT, bestDist);
    }

    private double refineLocalT(Segment seg, double px, double py, double startT) {
        double t = startT;
        for (int iter = 0; iter < 5; iter++) {
            Pair pt = Segments.eval(seg, t);
            Pair d1 = Segments.deriv(seg, t);
            Pair d2 = Segments.deriv2(seg, t);
            double dx = pt.getX() - px, dy = pt.getY() - py;
            double f = dx * d1.getX() + dy * d1.getY();
            double fd = (d1.getX() * d1.getX() + d1.getY() * d1.getY()) + (dx * d2.getX() + dy * d2.getY());
            if (Math.abs(fd) < 1e-12) break;
            double dt = f / fd;
            t -= dt;
            if (t < 0) t = 0;
            if (t > 1) t = 1;
            if (Math.abs(dt) < 1e-8) break;
        }
        return t;
    }

    // ═══════════════════════════════════════════════════════════
    // AABB
    // ═══════════════════════════════════════════════════════════

    @Override
    public AABB aabb(Curve curve) {
        AABB total = null;
        for (Segment seg : curve.getSegments()) {
            AABB box = Segments.aabb(seg);
            total = (total == null) ? box : total.merge(box);
        }
        return total == null ? new AABB(0, 0, 0, 0) : total;
    }

    @Override
    @Deprecated
    public AABB aabb(Curve curve, int idx) {
        List<ControlPoint> points = curve.getPoints();
        int size = points.size();
        if (idx < 0 || idx >= size) throw new IndexOutOfBoundsException("Index out of range");
        ControlPoint p = points.get(idx);
        AABB total = new AABB(p.getX(), p.getY(), p.getX(), p.getY());
        if (size == 1) {
            return total;
        }
        if (size == 2) {
            Segment seg = curve.getSegment(0);
            return Segments.aabb(seg);
        }

        boolean closed = curve.isClosed();

        if (idx == 0 && !closed) {
            Segment seg = curve.getSegment(0);
            return Segments.aabb(seg);
        }

        if (idx == size - 1 && !closed) {
            ControlPoint p1 = points.get(idx - 1);
            ControlPoint p2 = points.get(idx);
            return Segments.aabb(Segment.of(p1, p2));
        }

        List<Segment> segments = curve.getSegments();
        int segCount = segments.size();

        int leftSegIdx = idx - 1;
        if (closed && idx == 0) {
            leftSegIdx = segCount - 1;
        }
        if (leftSegIdx >= 0 && leftSegIdx < segCount) {
            AABB box = Segments.aabb(segments.get(leftSegIdx));
            total = total.merge(box);
        }

        int rightSegIdx = idx;
        if (closed && idx == size - 1) {
            rightSegIdx = segCount - 1;
        }
        if (rightSegIdx >= 0 && rightSegIdx < segCount) {
            AABB box = Segments.aabb(segments.get(rightSegIdx));
            total = total.merge(box);
        }

        return total;
    }

    @Override
    public AABB aabbRange(Curve curve, int fromSeg, int toSeg) {
        int segCount = curve.getSegmentCount();
        if (fromSeg < 0 || toSeg >= segCount || fromSeg > toSeg) {
            throw new IndexOutOfBoundsException(
                    "range [" + fromSeg + ", " + toSeg + "] out of [0, " + segCount + ")");
        }
        AABB total = null;
        for (int i = fromSeg; i <= toSeg; i++) {
            AABB box = Segments.aabb(curve.getSegment(i));
            total = (total == null) ? box : total.merge(box);
        }
        return total;
    }

    // ═══════════════════════════════════════════════════════════
    // 变换
    // ═══════════════════════════════════════════════════════════

    @Override
    public Curve translate(Curve curve, double dx, double dy) {
        List<ControlPoint> newPts = new ArrayList<>();
        for (ControlPoint p : curve.getPoints()) {
            newPts.add(ControlPoint.builder()
                    .x(p.getX() + dx).y(p.getY() + dy)
                    .dx1(p.getDx1()).dy1(p.getDy1())
                    .dx2(p.getDx2()).dy2(p.getDy2())
                    .continuity(p.getContinuity())
                    .build());
        }
        return new Curve(newPts, curve.isClosed());
    }

    @Override
    public Curve scale(Curve curve, double sx, double sy, double cx, double cy) {
        List<ControlPoint> newPts = new ArrayList<>();
        for (ControlPoint p : curve.getPoints()) {
            double nx = cx + (p.getX() - cx) * sx, ny = cy + (p.getY() - cy) * sy;
            newPts.add(ControlPoint.builder()
                    .x(nx).y(ny)
                    .dx1(p.getDx1() * sx).dy1(p.getDy1() * sy)
                    .dx2(p.getDx2() * sx).dy2(p.getDy2() * sy)
                    .continuity(p.getContinuity())
                    .build());
        }
        return new Curve(newPts, curve.isClosed());
    }

    @Override
    public Curve transform(Curve curve, double a, double b, double c, double d, double tx, double ty) {
        List<ControlPoint> newPts = new ArrayList<>();
        for (ControlPoint p : curve.getPoints()) {
            double px = p.getX() * a + p.getY() * c + tx;
            double py = p.getX() * b + p.getY() * d + ty;
            double dx1 = p.getDx1() * a + p.getDy1() * c;
            double dy1 = p.getDx1() * b + p.getDy1() * d;
            double dx2 = p.getDx2() * a + p.getDy2() * c;
            double dy2 = p.getDx2() * b + p.getDy2() * d;
            newPts.add(new ControlPoint(px, py, dx1, dy1, dx2, dy2, p.getContinuity()));
        }
        return new Curve(newPts, curve.isClosed());
    }

    // ═══════════════════════════════════════════════════════════
    // 切分
    // ═══════════════════════════════════════════════════════════

    @Override
    public void cut(Curve curve, double t, Curve out1, Curve out2) {
        if (t <= 0) {
            out1.setPoints(new ArrayList<>()); out1.setClosed(false);
            out2.setPoints(copyControlPoints(curve.getPoints())); out2.setClosed(curve.isClosed());
            return;
        }
        if (t >= 1) {
            out1.setPoints(copyControlPoints(curve.getPoints())); out1.setClosed(curve.isClosed());
            out2.setPoints(new ArrayList<>()); out2.setClosed(false);
            return;
        }

        IndexedSegment is = curve.segmentAt(t);
        int idx = is.getIndex();
        double localT = is.getLocal();
        Segment seg = is.getSegment();

        Segment leftSeg = new Segment();
        Segment rightSeg = new Segment();
        Segments.split(seg, localT, leftSeg, rightSeg);

        List<ControlPoint> leftPoints = new ArrayList<>();
        for (int i = 0; i < idx; i++) {
            leftPoints.add(curve.getPoints().get(i).copy());
        }

        ControlPoint leftStart = curve.getPoints().get(idx).copy();
        leftStart.setDx2(leftSeg.getB().getX() - leftSeg.getA().getX());
        leftStart.setDy2(leftSeg.getB().getY() - leftSeg.getA().getY());
        leftPoints.add(leftStart);

        double splitX = leftSeg.getD().getX(), splitY = leftSeg.getD().getY();
        double inDx = leftSeg.getC().getX() - leftSeg.getD().getX();
        double inDy = leftSeg.getC().getY() - leftSeg.getD().getY();
        double outDx = rightSeg.getB().getX() - rightSeg.getA().getX();
        double outDy = rightSeg.getB().getY() - rightSeg.getA().getY();

        ControlPoint splitPoint = ControlPoint.builder()
                .x(splitX).y(splitY)
                .dx1(inDx).dy1(inDy)
                .dx2(outDx).dy2(outDy)
                .continuity(Continuity.C1)
                .build();

        leftPoints.add(splitPoint);

        List<ControlPoint> rightPoints = new ArrayList<>();
        rightPoints.add(splitPoint.copy());

        ControlPoint rightEnd = curve.getPoints().get(idx + 1).copy();
        rightEnd.setDx1(rightSeg.getC().getX() - rightSeg.getD().getX());
        rightEnd.setDy1(rightSeg.getC().getY() - rightSeg.getD().getY());
        rightPoints.add(rightEnd);

        for (int i = idx + 2; i < curve.getPoints().size(); i++) {
            rightPoints.add(curve.getPoints().get(i).copy());
        }

        out1.setPoints(leftPoints);
        out1.setClosed(false);
        out2.setPoints(rightPoints);
        out2.setClosed(false);
    }

    @Override
    public void split(Curve curve, int idx, Curve out1, Curve out2) {
        List<ControlPoint> points = curve.getPoints();
        int n = points.size();
        if (idx < 0 || idx >= n) {
            throw new IndexOutOfBoundsException("Index " + idx + " out of bounds [0, " + (n - 1) + "]");
        }
        if (n < 2) {
            throw new IllegalStateException("Curve must have at least 2 points");
        }

        if (idx == 0) {
            out1.setPoints(new ArrayList<>());
            out1.setClosed(false);
            out2.setPoints(copyControlPoints(points));
            out2.setClosed(curve.isClosed());
        } else if (idx == n - 1) {
            out1.setPoints(copyControlPoints(points));
            out1.setClosed(curve.isClosed());
            out2.setPoints(new ArrayList<>());
            out2.setClosed(false);
        } else {
            List<ControlPoint> leftPoints = copyControlPoints(points.subList(0, idx + 1));
            List<ControlPoint> rightPoints = copyControlPoints(points.subList(idx, n));
            out1.setPoints(leftPoints);
            out1.setClosed(false);
            out2.setPoints(rightPoints);
            out2.setClosed(false);
        }
    }

    // ═══════════════════════════════════════════════════════════
    // 拓扑编辑
    // ═══════════════════════════════════════════════════════════

    @Override
    public Curve insertPoint(Curve curve, double t) {
        IndexedSegment is = curve.segmentAt(t);
        return insertPoint(curve, is.getIndex(), is.getLocal());
    }

    @Override
    public Curve insertPoint(Curve curve, int segIdx, double localT) {
        List<ControlPoint> points = curve.getPoints();
        int segCount = points.size() - 1;
        if (segCount <= 0) throw new IllegalArgumentException("Curve has no segments");
        if (segIdx < 0 || segIdx >= segCount) {
            throw new IndexOutOfBoundsException(
                    "segIdx " + segIdx + " out of [0, " + segCount + ")");
        }
        if (localT < 0.0 || localT > 1.0) {
            throw new IndexOutOfBoundsException(
                    "localT " + localT + " out of [0, 1]");
        }

        int idx = segIdx;
        Segment seg = curve.getSegment(idx);
        Segment leftSeg = new Segment(null, null, null, null);
        Segment rightSeg = new Segment(null, null, null, null);
        Segments.split(seg, localT, leftSeg, rightSeg);

        double mx = leftSeg.getD().getX(), my = leftSeg.getD().getY();

        Pair leftDeriv = Segments.deriv(leftSeg, 1.0);
        Pair rightDeriv = Segments.deriv(rightSeg, 0.0);
        double dxOut = rightDeriv.getX() / 3.0, dyOut = rightDeriv.getY() / 3.0;
        double dxIn = -leftDeriv.getX() / 3.0, dyIn = -leftDeriv.getY() / 3.0;

        ControlPoint newAnchor = ControlPoint.builder()
                .x(mx).y(my)
                .dx2(dxOut).dy2(dyOut)
                .dx1(dxIn).dy1(dyIn)
                .continuity(Continuity.C1)
                .build();

        ControlPoint origStart = points.get(idx).copy();
        ControlPoint origEnd = points.get(idx + 1).copy();

        Pair startDeriv = Segments.deriv(leftSeg, 0.0);
        origStart.setDx2(startDeriv.getX() / 3.0);
        origStart.setDy2(startDeriv.getY() / 3.0);

        Pair endDeriv = Segments.deriv(rightSeg, 1.0);
        origEnd.setDx1(-endDeriv.getX() / 3.0);
        origEnd.setDy1(-endDeriv.getY() / 3.0);

        List<ControlPoint> newPoints = new ArrayList<>(points.size() + 1);
        for (int i = 0; i < idx; i++) newPoints.add(points.get(i).copy());
        newPoints.add(origStart);
        newPoints.add(newAnchor);
        newPoints.add(origEnd);
        for (int i = idx + 2; i < points.size(); i++) {
            newPoints.add(points.get(i).copy());
        }
        return new Curve(newPoints, curve.isClosed());
    }

    @Override
    public Curve deletePoint(Curve curve, int idx) {
        List<ControlPoint> points = curve.getPoints();
        int n = points.size();
        if (n < 2) throw new IllegalArgumentException("Curve has too few points");
        if (idx < 0 || idx >= n) throw new IndexOutOfBoundsException("Index out of range");

        if (n == 2) return curve;

        if (idx == 0) {
            List<ControlPoint> newPoints = new ArrayList<>(n - 1);
            for (int i = 1; i < n; i++) newPoints.add(points.get(i).copy());
            return new Curve(newPoints, curve.isClosed());
        }
        if (idx == n - 1) {
            List<ControlPoint> newPoints = new ArrayList<>(n - 1);
            for (int i = 0; i < n - 1; i++) newPoints.add(points.get(i).copy());
            return new Curve(newPoints, curve.isClosed());
        }

        int segCount = n - 1;
        double tStart = (double) (idx - 1) / segCount;
        double tEnd = (double) (idx + 1) / segCount;
        int samples = 20;
        double[] xs = new double[samples], ys = new double[samples];
        for (int i = 0; i < samples; i++) {
            double t = tStart + (tEnd - tStart) * i / (samples - 1);
            Pair p = eval(curve, t);
            xs[i] = p.getX();
            ys[i] = p.getY();
        }

        Curve fitted = CurveFitter.fit(xs, ys, 5.0, 1);
        List<ControlPoint> fittedPoints = fitted.getPoints();
        if (fittedPoints.size() != 2) return curve;

        List<ControlPoint> newPoints = new ArrayList<>(n - 1);
        for (int i = 0; i < idx - 1; i++) newPoints.add(points.get(i).copy());
        newPoints.add(fittedPoints.get(0).copy());
        newPoints.add(fittedPoints.get(1).copy());
        for (int i = idx + 2; i < n; i++) newPoints.add(points.get(i).copy());
        return new Curve(newPoints, curve.isClosed());
    }

    @Override
    public Curve reverse(Curve curve) {
        List<ControlPoint> original = curve.getPoints();
        List<ControlPoint> reversed = new ArrayList<>(original.size());
        for (int i = original.size() - 1; i >= 0; i--) {
            ControlPoint p = original.get(i);
            reversed.add(ControlPoint.builder()
                    .x(p.getX()).y(p.getY())
                    .dx2(p.getDx1()).dy2(p.getDy1())
                    .dx1(p.getDx2()).dy1(p.getDy2())
                    .continuity(p.getContinuity())
                    .build());
        }
        return new Curve(reversed, curve.isClosed());
    }

    // ═══════════════════════════════════════════════════════════
    // 合并
    // ═══════════════════════════════════════════════════════════

    @Override
    public Curve join(Curve left, Curve right) {
        List<ControlPoint> lp = left.getPoints(), rp = right.getPoints();
        if (lp.isEmpty() || rp.isEmpty()) throw new IllegalArgumentException("Cannot join empty curves");

        ControlPoint leftEnd = lp.get(lp.size() - 1);
        ControlPoint rightStart = rp.get(0);
        double mx = (leftEnd.getX() + rightStart.getX()) / 2, my = (leftEnd.getY() + rightStart.getY()) / 2;

        Pair leftDeriv = deriv(left, 1.0);
        Pair rightDeriv = deriv(right, 0.0);

        ControlPoint merged = ControlPoint.builder()
                .x(mx)
                .y(my)
                .dx2(rightDeriv.getX() / 3.0)
                .dy2(rightDeriv.getY() / 3.0)
                .dx1(-leftDeriv.getX() / 3.0)
                .dy1(-leftDeriv.getY() / 3.0)
                .continuity(Continuity.C1)
                .build();

        List<ControlPoint> all = new ArrayList<>(lp.size() + rp.size() - 1);
        for (int i = 0; i < lp.size() - 1; i++) {
            all.add(lp.get(i).copy());
        }
        all.add(merged);
        for (int i = 1; i < rp.size(); i++) {
            all.add(rp.get(i).copy());
        }
        return new Curve(all, false);
    }


    // ═══════════════════════════════════════════════════════════
    // 重构 / 拟合
    // ═══════════════════════════════════════════════════════════

    @Override
    public Curve fit(double[] xs, double[] ys, double maxError, int maxSeg) {
        return CurveFitter.fit(xs, ys, maxError, maxSeg);
    }

    @Override
    public Curve reform(Curve curve, int count) {
        int current = curve.getPoints().size();
        if (count < 2) throw new IllegalArgumentException("At least 2 control points required");
        if (count == current) return curve;

        if (count > current) {
            int targetSegs = count - 1;
            List<ControlPoint> allPoints = new ArrayList<>();

            List<ControlPoint> initialPoints = new ArrayList<>(current);
            for (ControlPoint p : curve.getPoints()) initialPoints.add(p.copy());
            Curve remaining = new Curve(initialPoints, curve.isClosed());

            double prevT = 0.0;
            for (int i = 1; i <= targetSegs; i++) {
                double nextT = (double) i / targetSegs;
                double localT = (nextT - prevT) / (1 - prevT);

                if (localT <= 0.0 || localT >= 1.0) {
                    List<ControlPoint> rpts = remaining.getPoints();
                    if (allPoints.isEmpty()) {
                        allPoints.addAll(rpts);
                    } else {
                        allPoints.remove(allPoints.size() - 1);
                        allPoints.addAll(rpts);
                    }
                    break;
                }

                Curve left = new Curve(Arrays.asList(new ControlPoint(), new ControlPoint()), false);
                Curve right = new Curve(Arrays.asList(new ControlPoint(), new ControlPoint()), false);
                cut(remaining, localT, left, right);

                List<ControlPoint> leftPts = left.getPoints();
                if (leftPts.isEmpty()) break;

                if (allPoints.isEmpty()) {
                    allPoints.addAll(leftPts);
                } else {
                    allPoints.remove(allPoints.size() - 1);
                    allPoints.addAll(leftPts);
                }
                remaining = right;
                prevT = nextT;
            }
            return new Curve(allPoints, curve.isClosed());
        } else {
            int targetSeg = count - 1;
            int samples = Math.max(8, targetSeg * 4);
            double[] xs = new double[samples], ys = new double[samples];
            for (int i = 0; i < samples; i++) {
                double t = (double) i / (samples - 1);
                Pair p = eval(curve, t);
                xs[i] = p.getX();
                ys[i] = p.getY();
            }
            Curve fitted = CurveFitter.fit(xs, ys, 0.0, targetSeg);
            List<ControlPoint> newPts = new ArrayList<>();
            for (ControlPoint cp : fitted.getPoints()) newPts.add(cp.copy());
            return new Curve(newPts, curve.isClosed());
        }
    }

    @Override
    public Curve reformRange(Curve curve, int fromSeg, int toSeg, int newSegCount) {
        int segCount = curve.getSegmentCount();
        if (fromSeg < 0 || toSeg >= segCount || fromSeg > toSeg) {
            throw new IndexOutOfBoundsException(
                    "range [" + fromSeg + ", " + toSeg + "] out of [0, " + segCount + ")");
        }
        if (newSegCount < 1) {
            throw new IllegalArgumentException("newSegCount must be >= 1");
        }

        List<ControlPoint> points = curve.getPoints();

        // 整条非闭合曲线——直接走 reform
        if (fromSeg == 0 && toSeg == segCount - 1 && !curve.isClosed()) {
            return reform(curve, newSegCount + 1);
        }

        // 提取子曲线（锚点 [fromSeg, toSeg+1]）
        List<ControlPoint> subPts = new ArrayList<>();
        for (int i = fromSeg; i <= toSeg + 1; i++) {
            subPts.add(points.get(i).copy());
        }
        Curve sub = new Curve(subPts, false);

        // 采样范围曲线
        int samples = Math.max(8, newSegCount * 4);
        double[] xs = new double[samples], ys = new double[samples];
        for (int i = 0; i < samples; i++) {
            double t = (double) i / (samples - 1);
            Pair p = eval(sub, t);
            xs[i] = p.getX();
            ys[i] = p.getY();
        }

        // 拟合
        Curve fitted = CurveFitter.fit(xs, ys, 0.0, newSegCount);
        List<ControlPoint> fittedPts = fitted.getPoints();
        if (fittedPts.isEmpty()) {
            throw new IllegalStateException("fit produced empty curve");
        }

        // 边界锚点的连续性继承原值
        ControlPoint firstOrig = points.get(fromSeg);
        ControlPoint lastOrig  = points.get(toSeg + 1);

        ControlPoint firstNew = fittedPts.get(0).copy();
        firstNew.setContinuity(firstOrig.getContinuity());

        ControlPoint lastNew = fittedPts.get(fittedPts.size() - 1).copy();
        lastNew.setContinuity(lastOrig.getContinuity());

        // 组装：前段 + 拟合段 + 后段
        List<ControlPoint> all = new ArrayList<>();
        for (int i = 0; i < fromSeg; i++) {
            all.add(points.get(i).copy());
        }
        all.add(firstNew);
        for (int i = 1; i < fittedPts.size() - 1; i++) {
            all.add(fittedPts.get(i).copy());
        }
        if (fittedPts.size() > 1) {
            all.add(lastNew);
        }
        for (int i = toSeg + 2; i < points.size(); i++) {
            all.add(points.get(i).copy());
        }

        return new Curve(all, curve.isClosed());
    }

    // ═══════════════════════════════════════════════════════════
    // 偏移
    // ═══════════════════════════════════════════════════════════

    @Override
    public Curve offset(Curve curve, double distance) {
        int segCount = curve.getSegmentCount();
        int samplePerSeg = 20;
        int totalSamples = segCount * samplePerSeg + 1;
        double[] xs = new double[totalSamples], ys = new double[totalSamples];
        for (int i = 0; i < totalSamples; i++) {
            double t = (double) i / (totalSamples - 1);
            Pair pt = eval(curve, t);
            Pair normal = normal(curve, t);
            xs[i] = pt.getX() + distance * normal.getX();
            ys[i] = pt.getY() + distance * normal.getY();
        }
        return CurveFitter.fit(xs, ys, 0.1, 200);
    }

    @Override
    public Curve offsetRange(Curve curve, int fromSeg, int toSeg, double distance) {
        int segCount = curve.getSegmentCount();
        if (fromSeg < 0 || toSeg >= segCount || fromSeg > toSeg) {
            throw new IndexOutOfBoundsException(
                    "range [" + fromSeg + ", " + toSeg + "] out of [0, " + segCount + ")");
        }

        List<ControlPoint> points = curve.getPoints();

        // 提取子曲线（锚点 [fromSeg, toSeg+1]）
        List<ControlPoint> subPts = new ArrayList<>();
        for (int i = fromSeg; i <= toSeg + 1; i++) {
            subPts.add(points.get(i).copy());
        }
        Curve sub = new Curve(subPts, false);

        // 采样范围内曲线并偏移
        int subSegCount = sub.getSegmentCount();
        int samplePerSeg = 20;
        int totalSamples = subSegCount * samplePerSeg + 1;
        double[] xs = new double[totalSamples], ys = new double[totalSamples];
        for (int i = 0; i < totalSamples; i++) {
            double t = (double) i / (totalSamples - 1);
            Pair pt = eval(sub, t);
            Pair n = normal(sub, t);
            xs[i] = pt.getX() + distance * n.getX();
            ys[i] = pt.getY() + distance * n.getY();
        }
        Curve offsetSub = CurveFitter.fit(xs, ys, 0.1, 200);
        List<ControlPoint> offsetPts = offsetSub.getPoints();

        // 组装：前段 [0, fromSeg) + 偏移段全部 + 后段 (toSeg+1, n)
        List<ControlPoint> all = new ArrayList<>();
        for (int i = 0; i < fromSeg; i++) {
            all.add(points.get(i).copy());
        }
        for (ControlPoint cp : offsetPts) {
            all.add(cp.copy());
        }
        for (int i = toSeg + 2; i < points.size(); i++) {
            all.add(points.get(i).copy());
        }

        return new Curve(all, curve.isClosed());
    }

    // ═══════════════════════════════════════════════════════════
    // 内部
    // ═══════════════════════════════════════════════════════════

    private List<ControlPoint> copyControlPoints(List<ControlPoint> pts) {
        List<ControlPoint> copy = new ArrayList<>(pts.size());
        for (ControlPoint p : pts) copy.add(p.copy());
        return copy;
    }
}