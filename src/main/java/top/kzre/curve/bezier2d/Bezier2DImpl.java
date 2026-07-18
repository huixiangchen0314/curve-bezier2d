package top.kzre.curve.bezier2d;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Bezier2DImpl implements Bezier2D.Spec {

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
    public AABB aabb(Curve curve) {
        AABB total = null;
        for (Segment seg : curve.getSegments()) {
            AABB box = Segments.aabb(seg);
            total = (total == null) ? box : total.merge(box);
        }
        return total == null ? new AABB(0,0,0,0) : total;
    }

    @Override
    public Curve translate(Curve curve, double dx, double dy) {
        List<ControlPoint> newPts = new ArrayList<>();
        for (ControlPoint p : curve.getPoints()) {
            newPts.add(new ControlPoint()
                    .setX(p.getX()+dx).setY(p.getY()+dy)
                    .setDx1(p.getDx1()).setDy1(p.getDy1())
                    .setDx2(p.getDx2()).setDy2(p.getDy2())
                    .setG1(p.isG1()));
        }
        return new Curve(newPts, curve.isClosed());
    }

    @Override
    public Curve scale(Curve curve, double sx, double sy, double cx, double cy) {
        List<ControlPoint> newPts = new ArrayList<>();
        for (ControlPoint p : curve.getPoints()) {
            double nx = cx + (p.getX()-cx)*sx, ny = cy + (p.getY()-cy)*sy;
            newPts.add(new ControlPoint()
                    .setX(nx).setY(ny)
                    .setDx1(p.getDx1()*sx).setDy1(p.getDy1()*sy)
                    .setDx2(p.getDx2()*sx).setDy2(p.getDy2()*sy)
                    .setG1(p.isG1()));
        }
        return new Curve(newPts, curve.isClosed());
    }

    @Override
    public void divide(Curve curve, int idx, Curve out1, Curve out2) {
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
            newPts.add(new ControlPoint(px, py, dx1, dy1, dx2, dy2, p.isG1()));
        }
        return new Curve(newPts, curve.isClosed());
    }


    @Override
    public void split(Curve curve, double t, Curve out1, Curve out2) {
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

        Segment leftSeg = new Segment(null,null,null,null);
        Segment rightSeg = new Segment(null,null,null,null);
        Segments.split(seg, localT, leftSeg, rightSeg);

        List<ControlPoint> leftPoints = new ArrayList<>();
        for (int i = 0; i < idx; i++) leftPoints.add(curve.getPoints().get(i).copy());

        ControlPoint leftStart = curve.getPoints().get(idx).copy();
        leftStart.setDx2(leftSeg.getB().getX()-leftSeg.getA().getX());
        leftStart.setDy2(leftSeg.getB().getY()-leftSeg.getA().getY());
        leftPoints.add(leftStart);

        double splitX = leftSeg.getD().getX(), splitY = leftSeg.getD().getY();
        double inDx = leftSeg.getC().getX() - leftSeg.getD().getX();
        double inDy = leftSeg.getC().getY() - leftSeg.getD().getY();
        double outDx = rightSeg.getB().getX() - rightSeg.getA().getX();
        double outDy = rightSeg.getB().getY() - rightSeg.getA().getY();

        ControlPoint splitPoint = new ControlPoint()
                .setX(splitX).setY(splitY)
                .setDx1(inDx).setDy1(inDy)
                .setDx2(outDx).setDy2(outDy)
                .setG1(true);
        splitPoint.applyConstraints();
        leftPoints.add(splitPoint);

        List<ControlPoint> rightPoints = new ArrayList<>();
        rightPoints.add(splitPoint.copy());

        ControlPoint rightEnd = curve.getPoints().get(idx+1).copy();
        rightEnd.setDx1(rightSeg.getC().getX() - rightSeg.getD().getX());
        rightEnd.setDy1(rightSeg.getC().getY() - rightSeg.getD().getY());
        rightPoints.add(rightEnd);

        for (int i = idx+2; i < curve.getPoints().size(); i++) rightPoints.add(curve.getPoints().get(i).copy());

        out1.setPoints(leftPoints); out1.setClosed(false);
        out2.setPoints(rightPoints); out2.setClosed(false);
    }

    private List<ControlPoint> copyControlPoints(List<ControlPoint> pts) {
        List<ControlPoint> copy = new ArrayList<>(pts.size());
        for (ControlPoint p : pts) copy.add(p.copy());
        return copy;
    }

    @Override
    public Curve fit(double[] xs, double[] ys, double maxError, int maxSeg) {
        return CurveFitter.fit(xs, ys, maxError, maxSeg);
    }

    @Override
    public Curve join(Curve left, Curve right) {
        List<ControlPoint> lp = left.getPoints(), rp = right.getPoints();
        if (lp.isEmpty() || rp.isEmpty()) throw new IllegalArgumentException("Cannot join empty curves");

        ControlPoint leftEnd = lp.get(lp.size()-1);
        ControlPoint rightStart = rp.get(0);
        double mx = (leftEnd.getX()+rightStart.getX())/2, my = (leftEnd.getY()+rightStart.getY())/2;

        Pair leftDeriv = deriv(left, 1.0);
        Pair rightDeriv = deriv(right, 0.0);

        ControlPoint merged = new ControlPoint()
                .setX(mx).setY(my)
                .setDx2(leftDeriv.getX()/3.0).setDy2(leftDeriv.getY()/3.0)          // 出切
                .setDx1(-rightDeriv.getX()/3.0).setDy1(-rightDeriv.getY()/3.0)     // 入切反向
                .setG1(true);
        merged.applyConstraints();

        List<ControlPoint> all = new ArrayList<>(lp.size()+rp.size()-1);
        for (int i=0; i<lp.size()-1; i++) all.add(lp.get(i).copy());
        all.add(merged);
        for (int i=1; i<rp.size(); i++) all.add(rp.get(i).copy());
        return new Curve(all, false);
    }

    @Override
    public void insertPoint(Curve curve, double t) {
        List<ControlPoint> points = curve.getPoints();
        int segCount = points.size()-1;
        if (segCount <= 0) throw new IllegalArgumentException("Curve has no segments");

        IndexedSegment is = curve.segmentAt(t);
        int idx = is.getIndex();
        double localT = is.getLocal();

        Segment seg = is.getSegment();
        Segment leftSeg = new Segment(null,null,null,null);
        Segment rightSeg = new Segment(null,null,null,null);
        Segments.split(seg, localT, leftSeg, rightSeg);

        double mx = leftSeg.getD().getX(), my = leftSeg.getD().getY();

        Pair leftDeriv = Segments.deriv(leftSeg, 1.0);
        Pair rightDeriv = Segments.deriv(rightSeg, 0.0);
        double dxOut = rightDeriv.getX()/3.0, dyOut = rightDeriv.getY()/3.0;   // 出切
        double dxIn  = -leftDeriv.getX()/3.0,  dyIn  = -leftDeriv.getY()/3.0;  // 入切反向

        ControlPoint newAnchor = new ControlPoint()
                .setX(mx).setY(my)
                .setDx2(dxOut).setDy2(dyOut)
                .setDx1(dxIn).setDy1(dyIn)
                .setG1(true);
        newAnchor.applyConstraints();

        ControlPoint origStart = points.get(idx).copy();
        ControlPoint origEnd   = points.get(idx+1).copy();

        Pair startDeriv = Segments.deriv(leftSeg, 0.0);
        origStart.setDx2(startDeriv.getX()/3.0);
        origStart.setDy2(startDeriv.getY()/3.0);

        Pair endDeriv = Segments.deriv(rightSeg, 1.0);
        origEnd.setDx1(-endDeriv.getX()/3.0);   // 入切反向
        origEnd.setDy1(-endDeriv.getY()/3.0);

        List<ControlPoint> newPoints = new ArrayList<>(points.size()+1);
        for (int i=0; i<idx; i++) newPoints.add(points.get(i).copy());
        newPoints.add(origStart);
        newPoints.add(newAnchor);
        newPoints.add(origEnd);
        for (int i=idx+2; i<points.size(); i++) newPoints.add(points.get(i).copy());
        curve.setPoints(newPoints);
    }

    @Override
    public void deletePoint(Curve curve, int idx) {
        List<ControlPoint> points = curve.getPoints();
        int n = points.size();
        if (n < 2) throw new IllegalArgumentException("Curve has too few points");
        if (idx < 0 || idx >= n) throw new IndexOutOfBoundsException("Index out of range");

        if (n == 2) return;
        if (idx == 0) { points.remove(0); return; }
        if (idx == n-1) { points.remove(n-1); return; }

        int segCount = n-1;
        double tStart = (double)(idx-1)/segCount, tEnd = (double)(idx+1)/segCount;
        int samples = 20;
        double[] xs = new double[samples], ys = new double[samples];
        for (int i=0; i<samples; i++) {
            double t = tStart + (tEnd-tStart)*i/(samples-1);
            Pair p = eval(curve, t);
            xs[i]=p.getX(); ys[i]=p.getY();
        }

        Curve fitted = CurveFitter.fit(xs, ys, 5.0, 1);
        List<ControlPoint> fittedPoints = fitted.getPoints();
        if (fittedPoints.size() != 2) return;

        List<ControlPoint> newPoints = new ArrayList<>(n-1);
        for (int i=0; i<idx-1; i++) newPoints.add(points.get(i).copy());
        newPoints.add(fittedPoints.get(0).copy());
        newPoints.add(fittedPoints.get(1).copy());
        for (int i=idx+2; i<n; i++) newPoints.add(points.get(i).copy());
        curve.setPoints(newPoints);
    }

    @Override
    public Curve reverse(Curve curve) {
        List<ControlPoint> original = curve.getPoints();
        List<ControlPoint> reversed = new ArrayList<>(original.size());
        for (int i=original.size()-1; i>=0; i--) {
            ControlPoint p = original.get(i);
            reversed.add(new ControlPoint()
                    .setX(p.getX()).setY(p.getY())
                    .setDx2(p.getDx1()).setDy2(p.getDy1())
                    .setDx1(p.getDx2()).setDy1(p.getDy2())
                    .setG1(p.isG1()));
        }
        return new Curve(reversed, curve.isClosed());
    }

    @Override
    public Pair unitTangent(Curve curve, double t) {
        Pair d = deriv(curve, t);
        double len = Math.hypot(d.getX(), d.getY());
        return len<1e-12 ? new Pair(0,0) : new Pair(d.getX()/len, d.getY()/len);
    }

    @Override
    public Pair unitNormal(Curve curve, double t) {
        Pair tan = unitTangent(curve, t);
        return new Pair(-tan.getY(), tan.getX());
    }

    @Override
    public double curvature(Curve curve, double t) {
        Pair d1 = deriv(curve, t), d2 = deriv2(curve, t);
        double cross = d1.getX()*d2.getY() - d1.getY()*d2.getX();
        double len = Math.hypot(d1.getX(), d1.getY());
        return len<1e-12 ? 0 : cross/(len*len*len);
    }

    @Override
    public Pair[] sample(Curve curve, int count) {
        if (count < 2) throw new IllegalArgumentException("count must be at least 2");
        Pair[] result = new Pair[count];
        for (int i=0; i<count; i++) result[i] = eval(curve, (double)i/(count-1));
        return result;
    }

    @Override
    public Curve offset(Curve curve, double distance) {
        int segCount = curve.getSegmentCount();
        int samplePerSeg = 20;
        int totalSamples = segCount*samplePerSeg+1;
        double[] xs = new double[totalSamples], ys = new double[totalSamples];
        for (int i=0; i<totalSamples; i++) {
            double t = (double)i/(totalSamples-1);
            Pair pt = eval(curve, t);
            Pair normal = unitNormal(curve, t);
            xs[i] = pt.getX() + distance*normal.getX();
            ys[i] = pt.getY() + distance*normal.getY();
        }
        return CurveFitter.fit(xs, ys, 0.1, 200);
    }

    @Override
    public void reform(Curve curve, int count) {
        int current = curve.getPoints().size();
        if (count < 2) throw new IllegalArgumentException("At least 2 control points required");
        if (count == current) return;

        if (count > current) {
            int targetSegs = count - 1;
            List<ControlPoint> allPoints = new ArrayList<>();
            Curve remaining = new Curve(new ArrayList<>(curve.getPoints()), curve.isClosed());
            double prevT = 0.0;
            for (int i = 1; i <= targetSegs; i++) {
                double nextT = (double) i / targetSegs;
                double localT = (nextT - prevT) / (1 - prevT);

                if (localT <= 0.0 || localT >= 1.0) {
                    List<ControlPoint> rpts = remaining.getPoints();
                    if (allPoints.isEmpty()) {
                        allPoints.addAll(rpts);
                    } else {
                        // 替换最后一个点（它可能是上一段的终点，手柄已被 split 更新）
                        allPoints.remove(allPoints.size() - 1);
                        allPoints.addAll(rpts);
                    }
                    break;
                }

                Curve left = new Curve(Arrays.asList(new ControlPoint(), new ControlPoint()), false);
                Curve right = new Curve(Arrays.asList(new ControlPoint(), new ControlPoint()), false);
                split(remaining, localT, left, right);

                List<ControlPoint> leftPts = left.getPoints();
                if (leftPts.isEmpty()) break;

                if (allPoints.isEmpty()) {
                    allPoints.addAll(leftPts);
                } else {
                    // 最后一个点（上一段的终点）的手柄已被 split 更新，替换之
                    allPoints.remove(allPoints.size() - 1);
                    allPoints.addAll(leftPts);
                }
                remaining = right;
                prevT = nextT;
            }
            curve.setPoints(allPoints);
        } else {
            int targetSeg = count - 1;
            int samples = Math.max(8, targetSeg*4);
            double[] xs = new double[samples], ys = new double[samples];
            for (int i=0; i<samples; i++) {
                double t = (double)i/(samples-1);
                Pair p = eval(curve, t);
                xs[i]=p.getX(); ys[i]=p.getY();
            }
            Curve fitted = CurveFitter.fit(xs, ys, 0.0, targetSeg);
            List<ControlPoint> newPts = new ArrayList<>();
            for (ControlPoint cp : fitted.getPoints()) newPts.add(cp.copy());
            curve.setPoints(newPts);
        }
    }

    @Override
    public ClosestPointResult closestPoint(Curve curve, Pair point) {
        double px=point.getX(), py=point.getY();
        double bestDist=Double.POSITIVE_INFINITY, bestGlobalT=0;
        Pair bestPt=null;
        List<Segment> segments = curve.getSegments();
        int totalSegs = segments.size();

        for (int i=0; i<totalSegs; i++) {
            Segment seg = segments.get(i);
            double localBestT=0, localBestDist=Double.POSITIVE_INFINITY;
            for (int s=0; s<=10; s++) {
                double t = s/10.0;
                Pair p = Segments.eval(seg, t);
                double d = Math.hypot(p.getX()-px, p.getY()-py);
                if (d < localBestDist) { localBestDist=d; localBestT=t; }
            }
            double refinedT = refineLocalT(seg, px, py, localBestT);
            Pair refinedPt = Segments.eval(seg, refinedT);
            double refinedDist = Math.hypot(refinedPt.getX()-px, refinedPt.getY()-py);
            if (refinedDist < bestDist) {
                bestDist = refinedDist;
                bestPt = refinedPt;
                bestGlobalT = (i + refinedT)/totalSegs;
            }
        }
        return new ClosestPointResult(bestPt, bestGlobalT, bestDist);
    }

    private double refineLocalT(Segment seg, double px, double py, double startT) {
        double t = startT;
        for (int iter=0; iter<5; iter++) {
            Pair pt = Segments.eval(seg, t);
            Pair d1 = Segments.deriv(seg, t);
            Pair d2 = Segments.deriv2(seg, t);
            double dx = pt.getX()-px, dy = pt.getY()-py;
            double f = dx*d1.getX() + dy*d1.getY();
            double fd = (d1.getX()*d1.getX()+d1.getY()*d1.getY()) + (dx*d2.getX()+dy*d2.getY());
            if (Math.abs(fd) < 1e-12) break;
            double dt = f/fd;
            t -= dt;
            if (t<0) t=0; if (t>1) t=1;
            if (Math.abs(dt) < 1e-8) break;
        }
        return t;
    }
}