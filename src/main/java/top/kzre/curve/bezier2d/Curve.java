package top.kzre.curve.bezier2d;

import lombok.Builder;
import lombok.Getter;
import lombok.ToString;

import java.util.ArrayList;
import java.util.List;


@Builder
@ToString
@Getter
public final class Curve {
    @Builder.Default
    private List<ControlPoint> points = new ArrayList<>();
    private boolean closed;

    public Curve(List<ControlPoint> controlPoints, boolean closed) {
        if (controlPoints.size() < 2) {
            throw new IllegalArgumentException();
        }
        this.points = new ArrayList<>(controlPoints);   // 可变副本
        this.closed = closed;
    }

    public Curve(List<ControlPoint> controlPoints) {
        this(controlPoints, false);
    }

    public Curve setPoints(List<ControlPoint> points) {
        this.points.clear();
        this.points.addAll(points);
        return this;
    }
    public Curve setClosed(boolean closed) {
        this.closed = closed;
        return this;
    }

    public int getDegree() {
        return points.size() - 1;
    }

    public Curve applyConstraints() {
        points.forEach(ControlPoint::applyConstraints);
        return this;
    }

    public Curve appendPoint(ControlPoint controlPoint) {
        points.add(controlPoint);
        return this;
    }

    public Curve insertPoint(ControlPoint controlPoint, int index) {
        points.add(index, controlPoint);
        return this;
    }

    public Curve removePoint(int index) {
        points.remove(index);
        return this;
    }

    /** 返回曲线包含的三次贝塞尔段的数量 */
    public int getSegmentCount() {
        int n = points.size();
        return closed ? n : n - 1;
    }


    public Segment getSegment(int idx) {
        int n = points.size();
        if (idx < 0 || idx >= getSegmentCount()) {
            throw new IndexOutOfBoundsException("Segment index out of range");
        }
        ControlPoint start = points.get(idx);
        ControlPoint end = points.get((idx + 1) % n);
        return Segment.of(start, end);
    }

    public List<Segment> getSegments() {
        int segCount = getSegmentCount();
        List<Segment> segments = new ArrayList<>(segCount);
        for (int i = 0; i < segCount; i++) {
            segments.add(getSegment(i));
        }
        return segments;
    }


    public IndexedSegment segmentAt(double t) {
        int totalSegs = getSegmentCount();
        if (totalSegs <= 0) {
            throw new IllegalArgumentException("Curve has no segments");
        }
        if (t <= 0) {
            return new IndexedSegment(getSegment(0), 0.0, 0);
        }
        if (t >= 1) {
            return new IndexedSegment(getSegment(totalSegs - 1), 1.0, totalSegs - 1);
        }
        double scaled = t * totalSegs;
        int idx = (int) Math.floor(scaled);
        if (idx >= totalSegs) idx = totalSegs - 1;
        double localT = scaled - idx;
        return new IndexedSegment(getSegment(idx), localT, idx);
    }
}
