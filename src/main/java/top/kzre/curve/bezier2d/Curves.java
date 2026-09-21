package top.kzre.curve.bezier2d;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * 影响范围曲线预设。
 *
 * <p>曲线定义在单位空间 {@code (0,1) → (1,0)}：
 * <ul>
 *   <li>x 轴 —— 归一化距离（0 = 中心，1 = 影响半径边缘）</li>
 *   <li>y 轴 —— 归一化强度（1 = 全强度，0 = 无影响）</li>
 * </ul>
 *
 * <p>用途：衰减编辑的 falloff 曲线——按距离加权。
 * 也用于任何需要「距离 → 权重」映射的场景。
 *
 * <p><b>收录范围：</b>只收录无法用简单闭式函数优雅表达的曲线。
 * linear / constant 等函数已足够——不在此列。
 *
 * <p>所有曲线均为单段、非闭合、无连续性约束。
 */
public final class Curves {
    private Curves() {}

    // ═══════════════════════════════════════════════
    // 内部构造
    // ═══════════════════════════════════════════════

    /**
     * 单段三次贝塞尔：(0,1) → (1,0)。
     * (c1x, c1y) —— 起点出切控制柄绝对坐标
     * (c2x, c2y) —— 终点入切控制柄绝对坐标
     */
    public static Curve cubic01(double c1x, double c1y, double c2x, double c2y) {
        ControlPoint p0 = ControlPoint.builder()
                .x(0.0).y(1.0)
                .dx1(0.0).dy1(0.0)
                .dx2(c1x).dy2(c1y - 1.0)
                .continuity(Continuity.NONE)
                .build();
        ControlPoint p1 = ControlPoint.builder()
                .x(1.0).y(0.0)
                .dx1(c2x - 1.0).dy1(c2y)
                .dx2(0.0).dy2(0.0)
                .continuity(Continuity.NONE)
                .build();
        return new Curve(Arrays.asList(p0, p1), false);
    }

    // ═══════════════════════════════════════════════
    // 四大基础预设
    // ═══════════════════════════════════════════════

    /**
     * 锐利 —— 对应 {@code (1-x)²} 的近似。
     *
     * <p>中心快速下降，中段趋平缓，边缘平稳到达 0。
     * 视觉：从中心出发立刻就能感到衰减。
     */
    public static Curve sharp() {
        return cubic01(0.33, 0.33, 0.67, 0.0);
    }

    /**
     * 平滑 —— 对应 smoothstep {@code 1-3x²+2x³} 的近似。
     *
     * <p>两端水平、中段陡降的对称 S 曲线。
     * 视觉：中心附近和边缘附近都很柔和，中段过渡明显。
     */
    public static Curve smooth() {
        return cubic01(0.5, 1.0, 0.5, 0.0);
    }

    /**
     * 球状 —— 对应 {@code sqrt(1-x²)}（四分之一圆）的近似。
     *
     * <p>起点切线水平（球顶），终点切线垂直（球边）。
     * 视觉：中心大范围保持高强度，接近边缘才急剧下落。
     */
    public static Curve sphere() {
        // 圆的 cubic Bezier 标准近似系数 0.5523
        return cubic01(0.5523, 1.0, 1.0, 0.5523);
    }

    /**
     * 根凸 —— 对应 {@code sqrt(1-x)} 的近似。
     *
     * <p>起点切线缓（斜率约 -0.5），终点切线陡。
     * 视觉：中心缓慢衰减，边缘陡峭收尾——比 Sphere 更早开始降。
     */
    public static Curve root() {
        return cubic01(0.2, 0.9, 1.0, 0.2);
    }

    // ═══════════════════════════════════════════════
    // 组合操作
    // ═══════════════════════════════════════════════

    /**
     * 反转 —— y 变为 1 - y。
     *
     * <p>把「中心强，边缘弱」的衰减曲线变成「中心弱，边缘强」，
     * 或反之。所有控制点的 y 坐标与 y 方向控制柄取反。
     */
    public static Curve invert(Curve curve) {
        List<ControlPoint> pts = curve.getPoints();
        List<ControlPoint> out = new ArrayList<>(pts.size());
        for (ControlPoint p : pts) {
            out.add(ControlPoint.builder()
                    .x(p.getX())
                    .y(1.0 - p.getY())
                    .dx1(p.getDx1())
                    .dy1(-p.getDy1())
                    .dx2(p.getDx2())
                    .dy2(-p.getDy2())
                    .continuity(p.getContinuity())
                    .build());
        }
        return new Curve(out, curve.isClosed());
    }

    /**
     * 横向镜像 —— x 变为 1 - x。
     *
     * <p>把「前急后缓」变成「前缓后急」。
     * 控制点顺序需反转——因为新曲线的方向相反。
     */
    public static Curve mirrorX(Curve curve) {
        List<ControlPoint> pts = curve.getPoints();
        List<ControlPoint> out = new ArrayList<>(pts.size());
        for (ControlPoint p : pts) {
            out.add(ControlPoint.builder()
                    .x(1.0 - p.getX())
                    .y(p.getY())
                    .dx1(-p.getDx1())
                    .dy1(p.getDy1())
                    .dx2(-p.getDx2())
                    .dy2(p.getDy2())
                    .continuity(p.getContinuity())
                    .build());
        }
        Collections.reverse(out);
        return new Curve(out, curve.isClosed());
    }
}