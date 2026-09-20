package top.kzre.curve.bezier2d;

import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.ToString;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@NoArgsConstructor
@ToString
public final class Curve {
    private List<ControlPoint> points;
    @Getter
    private boolean closed;

    public Curve(List<ControlPoint> points, boolean closed) {
        this.points = new ArrayList<>(points);
        this.closed = closed;
    }

    void setPoints(List<ControlPoint> points) {
        this.points = points;
    }

    void setClosed(boolean closed) {
        this.closed = closed;
    }

    public List<ControlPoint> getPoints() {
        return Collections.unmodifiableList(points);
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

    // ═══════════════════════════════════════════════════════════
    // 连续性约束
    // ═══════════════════════════════════════════════════════════

    /**
     * 对所有控制点应用连续性约束。
     *
     * <p>约束规则：
     * <ul>
     *   <li>{@link Continuity#NONE}：不处理（角点）</li>
     *   <li>{@link Continuity#G1}：入切方向与出切反向共线，大小保持原比例</li>
     *   <li>{@link Continuity#C1}：入切 = −出切（方向共线 + 大小相等）</li>
     *   <li>{@link Continuity#G2}：G1 + 曲率相等（通过调整入切长度实现）</li>
     *   <li>{@link Continuity#C2}：C1 + 二阶连续——曲率部分需要多控制点协作，
     *       当前只强制 C1 部分</li>
     * </ul>
     *
     * <p><b>处理顺序：</b>从左到右。G2 读取的邻居手柄反映的是「邻居已被处理」或
     * 「尚未处理」的状态——对大多数编辑场景可接受。
     */
    public void applyConstraints() {
        int n = points.size();
        for (int i = 0; i < n; i++) {
            applyConstraintAt(i);
        }
    }

    private void applyConstraintAt(int idx) {
        ControlPoint cp = points.get(idx);
        switch (cp.getContinuity()) {
            case NONE:
                break;
            case G1:
                applyG1(cp);
                break;
            case C1:
                applyC1(cp);
                break;
            case G2:
                applyG2(idx);
                break;
            case C2:
                applyC2(idx);
                break;
        }
    }

    // ─── G1 / C1 ──────────────────────────────────────────────

    /** G1：方向共线，大小保持原比例 */
    private void applyG1(ControlPoint cp) {
        double dx1 = cp.getDx1(), dy1 = cp.getDy1();
        double dx2 = cp.getDx2(), dy2 = cp.getDy2();
        double lenIn  = Math.hypot(dx1, dy1);
        double lenOut = Math.hypot(dx2, dy2);

        if (lenOut < 1e-12) {
            cp.setDx1(0).setDy1(0);
            return;
        }
        if (lenIn < 1e-12) {
            cp.setDx1(-dx2).setDy1(-dy2);
        } else {
            double factor = lenIn / lenOut;
            cp.setDx1(-dx2 * factor).setDy1(-dy2 * factor);
        }
    }

    /** C1：入切 = −出切 */
    private void applyC1(ControlPoint cp) {
        cp.setDx1(-cp.getDx2()).setDy1(-cp.getDy2());
    }

    // ─── G2 ──────────────────────────────────────────────────

    /**
     * G2：方向共线（G1）+ 曲率相等。
     *
     * <p><b>推导：</b>设出切 h_out = (dx2, dy2)，入切 h_in = -k·h_out（k > 0）。
     * 相邻段：
     * <ul>
     *   <li>左段末端曲率 κ_L = (2/3)·|h_out × Δ| / (k²·|h_out|³)</li>
     *   <li>右段起点曲率 κ_R = (2/3)·|h_out × Γ| / |h_out|³</li>
     * </ul>
     * 其中 Δ = 左段 P1 − 锚点位置，Γ = 右段 P2 − 锚点位置。
     * 令 κ_L = κ_R 解得 k = √(|h_out × Δ| / |h_out × Γ|)。
     *
     * <p>端点（非闭合首尾）无两侧信息——降级为 G1。
     */
    private void applyG2(int idx) {
        ControlPoint cp = points.get(idx);
        double dx2 = cp.getDx2(), dy2 = cp.getDy2();
        double lenOut = Math.hypot(dx2, dy2);
        if (lenOut < 1e-12) {
            cp.setDx1(0).setDy1(0);
            return;
        }

        int n = points.size();
        int prevIdx, nextIdx;
        if (closed) {
            prevIdx = (idx - 1 + n) % n;
            nextIdx = (idx + 1) % n;
        } else {
            if (idx == 0 || idx == n - 1) {
                applyG1(cp);
                return;
            }
            prevIdx = idx - 1;
            nextIdx = idx + 1;
        }

        ControlPoint prev = points.get(prevIdx);
        ControlPoint next = points.get(nextIdx);

        // 左段 P1 = prev 位置 + prev 出切
        double p1Lx = prev.getX() + prev.getDx2();
        double p1Ly = prev.getY() + prev.getDy2();
        // 右段 P2 = next 位置 + next 入切
        double p2Rx = next.getX() + next.getDx1();
        double p2Ry = next.getY() + next.getDy1();
        double cx = cp.getX(), cy = cp.getY();

        // Δ = 左段 P1 − c
        double dLx = p1Lx - cx, dLy = p1Ly - cy;
        // Γ = 右段 P2 − c
        double gRx = p2Rx - cx, gRy = p2Ry - cy;

        // |h_out × Δ|, |h_out × Γ|
        double crossL = Math.abs(dx2 * dLy - dy2 * dLx);
        double crossR = Math.abs(dx2 * gRy - dy2 * gRx);

        if (crossR < 1e-12) {
            // Γ 与 h_out 共线 → 曲率约束无解，降级为 G1
            applyG1(cp);
            return;
        }

        double kRatio = Math.sqrt(crossL / crossR);

        // 入切：方向 -h_out，长度 lenOut · kRatio
        cp.setDx1(-dx2 * kRatio).setDy1(-dy2 * kRatio);
    }

    // ─── C2 ──────────────────────────────────────────────────

    /**
     * C2：C1 + 二阶连续。
     *
     * <p>二阶连续要求 B''_left(1) = B''_right(0)，即
     * <pre>
     *   P1 − c − 2·h_in = P2' − c − 2·h_out
     * </pre>
     * 化简得约束 <pre> P1 − P2' = −2(1 + k)·h_out </pre>
     * 其中 P1 是左段 P1、P2' 是右段 P2、k 是入切与出切长度比。
     *
     * <p>这要求 P2' − P1 恰好与 h_out 共线——一般情况不成立，
     * <b>无法通过只调整本点的手柄实现</b>。完整 C2 需要同时调整相邻控制点。
     *
     * <p>当前策略：强制 C1 部分；二阶部分由未来的曲线级多点协作扩展。
     */
    private void applyC2(int idx) {
        throw new UnsupportedOperationException(
                "C2 continuity is not yet implemented at index " + idx
                        + "; use Continuity.C1 or handle C2 at curve-level");
    }
}