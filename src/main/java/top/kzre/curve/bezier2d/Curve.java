package top.kzre.curve.bezier2d;

import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.ToString;

import java.util.ArrayList;
import java.util.List;

@NoArgsConstructor
@ToString
@Getter
public final class Curve {
    private List<ControlPoint> points;
    private boolean closed;

    public Curve(List<ControlPoint> controlPoints, boolean closed) {
//        if (controlPoints.size() < 2) {
//            throw new IllegalArgumentException();
//        }
        this.points = new ArrayList<>(controlPoints);   // 可变副本
        this.closed = closed;
    }

    public Curve(List<ControlPoint> controlPoints) {
        this(controlPoints, false);
    }

    public Curve setPoints(List<ControlPoint> points) {
        this.points = points;
        return this;
    }
    public Curve setClosed(boolean closed) {
        this.closed = closed;
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

    /**
     * 深拷贝当前曲线。
     * 返回一个新的 Curve 对象，其内部控制点列表由每个控制点的副本组成，
     * 修改原曲线或新曲线均不会相互影响。
     */
    public Curve copy() {
        List<ControlPoint> copiedPoints = new ArrayList<>(points.size());
        for (ControlPoint p : points) {
            copiedPoints.add(p.copy());
        }
        return new Curve(copiedPoints, this.closed);
    }
}
