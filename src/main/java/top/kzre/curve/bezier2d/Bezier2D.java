package top.kzre.curve.bezier2d;

import java.util.List;

/**
 * 多段三次贝塞尔曲线操作门面。
 *
 * <p>本类提供对 {@link Curve} 的所有几何操作。曲线由若干控制点
 * ({@link ControlPoint}) 连接而成，段数由控制点数决定：
 * <ul>
 *   <li>非闭合曲线：段数 = 控制点数 - 1</li>
 *   <li>闭合曲线：段数 = 控制点数</li>
 * </ul>
 *
 * <p><b>参数 t 的约定：</b>除特别说明外，方法的 {@code t} 参数均为
 * <b>全局归一化参数</b>，范围 [0, 1]，与段无关。{@code t = 0} 对应曲线起点，
 * {@code t = 1} 对应终点。内部通过 {@link #segmentIndex} 将其映射为段索引 + 段内局部 t。
 *
 * <p><b>纯函数约定：</b>返回 {@link Curve} 的方法不修改入参，总是返回新对象。
 * 使用 out 参数的方法（{@code split}、{@code divide}）将结果写入调用方提供的空容器。
 */
public final class Bezier2D {
    private static final Spec impl = new Bezier2DImpl();

    private Bezier2D() {}

    /**
     * 将全局归一化参数 t 映射为段索引。
     *
     * <p>映射规则：{@code segIdx = floor(t * segCount)}。{@code t = 1.0}
     * 归入末段（{@code segCount - 1}），避免越界。
     *
     * @param curve 曲线
     * @param t     全局归一化参数，范围 [0, 1]
     * @return 段索引，范围 [0, segCount)
     * @throws IndexOutOfBoundsException {@code t} 超出 [0, 1]
     * @throws IllegalStateException     曲线没有段（控制点数不足）
     */
    public static int segmentIndex(Curve curve, double t) {
        if (t < 0.0 || t > 1.0) {
            throw new IndexOutOfBoundsException("t " + t + " out of [0, 1]");
        }
        int segCount = curve.getSegmentCount();
        if (segCount <= 0) {
            throw new IllegalStateException("curve has no segments");
        }
        if (t >= 1.0) return segCount - 1;
        return (int) Math.floor(t * segCount);
    }

    // ═══════════════════════════════════════════════════════════
    // 求值
    // ═══════════════════════════════════════════════════════════

    /**
     * 计算曲线上 t 处的坐标。
     *
     * @param curve 曲线
     * @param t     全局归一化参数，范围 [0, 1]
     * @return 曲线上的点坐标
     */
    public static Pair eval(Curve curve, double t) {
        return impl.eval(curve, t);
    }

    /**
     * 计算曲线上 t 处的一阶导数（切向量，未归一化）。
     *
     * <p>方向为曲线前进方向，模长与参数化速度成正比。
     * 需要单位切向量时用 {@link #tangent}。
     *
     * @param curve 曲线
     * @param t     全局归一化参数
     * @return 一阶导数向量
     */
    public static Pair deriv(Curve curve, double t) {
        return impl.deriv(curve, t);
    }

    /**
     * 计算曲线上 t 处的二阶导数。
     *
     * @param curve 曲线
     * @param t     全局归一化参数
     * @return 二阶导数向量
     */
    public static Pair deriv2(Curve curve, double t) {
        return impl.deriv2(curve, t);
    }

    /**
     * 计算曲线上 t 处的单位切向量。
     *
     * <p>当一阶导数为零（如尖点或退化段）时返回 {@code (0, 0)}。
     *
     * @param curve 曲线
     * @param t     全局归一化参数
     * @return 单位切向量
     */
    public static Pair tangent(Curve curve, double t) {
        return impl.tangent(curve, t);
    }

    /**
     * 计算曲线上 t 处的单位法向量（左手系，指向切线左侧）。
     *
     * <p>由单位切向量逆时针旋转 90° 得到：{@code normal = (-tangent.y, tangent.x)}。
     *
     * @param curve 曲线
     * @param t     全局归一化参数
     * @return 单位法向量
     */
    public static Pair normal(Curve curve, double t) {
        return impl.normal(curve, t);
    }

    /**
     * 计算曲线上 t 处的有符号曲率。
     *
     * <p>公式：{@code κ = (P' × P'') / |P'|³}。
     * 正负号表示弯曲方向（逆时针为正）。一阶导数为零时返回 0。
     *
     * @param curve 曲线
     * @param t     全局归一化参数
     * @return 有符号曲率
     */
    public static double curvature(Curve curve, double t) {
        return impl.curvature(curve, t);
    }

    /**
     * 均匀采样曲线，返回含首尾的 count 个点。
     *
     * <p>采样按全局参数 t 均匀分布：第 i 个点对应 {@code t = i / (count - 1)}。
     * 注意这与「等弧长采样」不同——均匀参数不意味着均匀间距。
     *
     * @param curve 曲线
     * @param count 采样点数，必须 >= 2
     * @return 长度等于 count 的点数组
     * @throws IllegalArgumentException {@code count < 2}
     */
    public static Pair[] sample(Curve curve, int count) {
        return impl.sample(curve, count);
    }

    // ═══════════════════════════════════════════════════════════
    // 最近点
    // ═══════════════════════════════════════════════════════════

    /**
     * 在整条曲线上查找距离 {@code point} 最近的点。
     *
     * <p>算法：每段先粗采样定位局部最优，再用牛顿迭代精化。
     *
     * @param curve 曲线
     * @param point 查询点
     * @return 最近点信息（位置、参数 t、距离）
     */
    public static ClosestPointResult closestPoint(Curve curve, Pair point) {
        return impl.closestPoint(curve, point);
    }

    /**
     * 在段范围 {@code [fromSeg, toSeg]} 内查找最近点。
     *
     * <p>闭区间。返回的 {@code t} 仍是全局归一化参数。
     *
     * @param curve   曲线
     * @param point   查询点
     * @param fromSeg 起始段索引（含）
     * @param toSeg   结束段索引（含）
     * @return 最近点信息
     * @throws IndexOutOfBoundsException 范围非法
     */
    public static ClosestPointResult closestPointRange(Curve curve, Pair point,
                                                       int fromSeg, int toSeg) {
        return impl.closestPointRange(curve, point, fromSeg, toSeg);
    }

    // ═══════════════════════════════════════════════════════════
    // AABB
    // ═══════════════════════════════════════════════════════════

    /**
     * 计算整条曲线的精确包围盒。
     *
     * <p>通过求解每段 X/Y 方向导数的零点得到，比控制点包围盒更紧。
     *
     * @param curve 曲线
     * @return 包围盒
     */
    public static AABB aabb(Curve curve) {
        return impl.aabb(curve);
    }

    /**
     * 计算单个锚点两侧段的包围盒。
     *
     * @param curve 曲线
     * @param idx   锚点索引
     * @return 包围盒
     * @deprecated 单锚点语义模糊——「锚点两侧」易与「锚点本身」混淆。
     *             新代码用 {@link #aabbRange}。
     */
    @Deprecated
    public static AABB aabb(Curve curve, int idx) {
        return impl.aabb(curve, idx);
    }

    /**
     * 计算段范围 {@code [fromSeg, toSeg]} 的精确包围盒。
     *
     * <p>闭区间。
     *
     * @param curve   曲线
     * @param fromSeg 起始段索引（含）
     * @param toSeg   结束段索引（含）
     * @return 包围盒
     * @throws IndexOutOfBoundsException 范围非法
     */
    public static AABB aabbRange(Curve curve, int fromSeg, int toSeg) {
        return impl.aabbRange(curve, fromSeg, toSeg);
    }

    // ═══════════════════════════════════════════════════════════
    // 拓扑编辑
    // ═══════════════════════════════════════════════════════════

    /**
     * 在全局归一化参数 t 处插入控制点。
     *
     * <p>等价于 {@code insertPoint(curve, segmentIndex(curve, t), localT)}——
     * 调用方已知段索引时用段内版本可省一次映射。
     *
     * <p>插入后曲线的几何形状不变（de Casteljau 精确分割），
     * 但控制点数 +1，段数 +1。
     *
     * @param curve 曲线
     * @param t     全局归一化参数
     * @return 插入后的新曲线
     */
    public static Curve insertPoint(Curve curve, double t) {
        return impl.insertPoint(curve, t);
    }

    /**
     * 在段 {@code segIdx} 的段内参数 {@code localT} 处插入控制点。
     *
     * <p>段内参数 {@code localT ∈ [0, 1]}——0 对应段起点，1 对应段终点。
     *
     * @param curve  曲线
     * @param segIdx 段索引，范围 [0, segCount)
     * @param localT 段内参数，范围 [0, 1]
     * @return 插入后的新曲线
     * @throws IndexOutOfBoundsException {@code segIdx} 或 {@code localT} 越界
     */
    public static Curve insertPoint(Curve curve, int segIdx, double localT) {
        return impl.insertPoint(curve, segIdx, localT);
    }

    /**
     * 删除指定索引的控制点。
     *
     * <p>删除行为依索引位置而异：
     * <ul>
     *   <li>端点（首/末）：直接删除，相邻段消失</li>
     *   <li>中间点：局部拟合——用新的一段近似原两段，尽量保持形状</li>
     * </ul>
     *
     * <p>控制点数不足（n=2）时返回原曲线。
     *
     * @param curve 曲线
     * @param idx   控制点索引
     * @return 删除后的新曲线
     * @throws IndexOutOfBoundsException 索引越界
     */
    public static Curve deletePoint(Curve curve, int idx) {
        return impl.deletePoint(curve, idx);
    }

    /**
     * 反转曲线方向。
     *
     * <p>几何形状不变，但 t=0 对应原终点，t=1 对应原起点。
     * 每个控制点的入/出手柄互换。
     *
     * @param curve 曲线
     * @return 反向后的新曲线
     */
    public static Curve reverse(Curve curve) {
        return impl.reverse(curve);
    }

    /**
     * 合并两条曲线为一条。
     *
     * <p>连接点位置取两端控制点的中点；切线由两条曲线在连接端的一阶导数推导，
     * 保证连接点 C1 连续。两条曲线必须都非闭合。
     *
     * @param left  左曲线（左端保留原方向）
     * @param right 右曲线（右端保留原方向）
     * @return 合并后的新曲线，非闭合
     * @throws IllegalArgumentException 任一曲线为空
     */
    public static Curve join(Curve left, Curve right) {
        return impl.join(left, right);
    }



    /**
     * 在全局归一化参数 t 处将曲线切分为左右两条。
     *
     * <p>结果写入调用方提供的两个空 {@link Curve} 容器。
     * 切分是 de Casteljau 精确分割，几何形状完全保持。
     *
     * <p>{@code t <= 0} 时左空右全；{@code t >= 1} 时左全右空。
     *
     * @param curve 曲线
     * @param t     全局归一化参数
     * @param out1  接收左半段（非闭合）
     * @param out2  接收右半段（非闭合）
     */
    public static void cut(Curve curve, double t, Curve out1, Curve out2) {
        impl.cut(curve, t, out1, out2);
    }

    /**
     * 在锚点索引 {@code idx} 处将曲线切分为左右两条。
     *
     * <p>切点必须是已存在的锚点——不支持从任意 t 切分。
     * 若需在曲线中间切分，先 {@link #insertPoint} 插入锚点，再调用本方法。
     *
     * <p>切分点作为共享锚点同时出现在两条曲线上（但对象独立）。
     * 左右两条均非闭合。
     *
     * @param curve 曲线
     * @param idx   锚点索引
     * @param out1  接收左侧（含切点）
     * @param out2  接收右侧（含切点）
     * @throws IndexOutOfBoundsException 索引越界
     * @throws IllegalStateException     控制点数不足
     */
    public static void split(Curve curve, int idx, Curve out1, Curve out2) {
        impl.split(curve, idx, out1, out2);
    }

    // ═══════════════════════════════════════════════════════════
    // 重构 / 拟合
    // ═══════════════════════════════════════════════════════════

    /**
     * 将整条曲线重采样为 {@code count} 个控制点。
     *
     * <p>策略：
     * <ul>
     *   <li>{@code count > 当前}：精确分割——形状完全保持，控制点增多</li>
     *   <li>{@code count < 当前}：拟合——形状近似，用更少的控制点逼近</li>
     *   <li>{@code count == 当前}：返回原曲线</li>
     * </ul>
     *
     * @param curve 曲线
     * @param count 目标控制点数，必须 >= 2
     * @return 重构后的新曲线
     * @throws IllegalArgumentException {@code count < 2}
     */
    public static Curve reform(Curve curve, int count) {
        return impl.reform(curve, count);
    }

    /**
     * 对段范围 {@code [fromSeg, toSeg]} 内的曲线重新采样为 {@code newSegCount} 段。
     *
     * <p>范围外的锚点完全保留（位置、手柄、连续性）。范围内的曲线被替换为
     * {@code newSegCount} 段拟合结果。边界的两个锚点位置保留，手柄按新形状重算，
     * 连续性字段继承原值。
     *
     * <p>闭区间。若范围为整条非闭合曲线，等价于
     * {@code reform(curve, newSegCount + 1)}。
     *
     * @param curve       曲线
     * @param fromSeg     起始段索引（含）
     * @param toSeg       结束段索引（含）
     * @param newSegCount 范围内新段数，>= 1
     * @return 重构后的新曲线
     * @throws IndexOutOfBoundsException 范围非法
     * @throws IllegalArgumentException  {@code newSegCount < 1}
     */
    public static Curve reformRange(Curve curve, int fromSeg, int toSeg, int newSegCount) {
        return impl.reformRange(curve, fromSeg, toSeg, newSegCount);
    }

    /**
     * 从离散点集拟合一条贝塞尔曲线。
     *
     * <p>点集通过 {@code xs}/{@code ys} 平行数组提供。拟合算法基于最小二乘，
     * 分段数由误差阈值和最大段数共同约束。
     *
     * @param xs        X 坐标数组
     * @param ys        Y 坐标数组，长度必须与 {@code xs} 相同
     * @param maxError  允许的最大拟合误差
     * @param maxSeg    最大段数上限
     * @return 拟合得到的曲线
     */
    public static Curve fit(double[] xs, double[] ys, double maxError, int maxSeg) {
        return impl.fit(xs, ys, maxError, maxSeg);
    }

    // ═══════════════════════════════════════════════════════════
    // 变换
    // ═══════════════════════════════════════════════════════════

    /**
     * 平移整条曲线。
     *
     * <p>所有控制点位置 +（dx, dy）。手柄向量是相对量，不受平移影响。
     *
     * @param curve 曲线
     * @param dx    X 方向平移量
     * @param dy    Y 方向平移量
     * @return 平移后的新曲线
     */
    public static Curve translate(Curve curve, double dx, double dy) {
        return impl.translate(curve, dx, dy);
    }

    /**
     * 以 {@code (cx, cy)} 为中心缩放曲线。
     *
     * <p>控制点位置按缩放比例变换；手柄向量按相同的线性分量缩放。
     *
     * @param curve 曲线
     * @param sx    X 方向缩放因子
     * @param sy    Y 方向缩放因子
     * @param cx    缩放中心 X
     * @param cy    缩放中心 Y
     * @return 缩放后的新曲线
     */
    public static Curve scale(Curve curve, double sx, double sy, double cx, double cy) {
        return impl.scale(curve, sx, sy, cx, cy);
    }

    /**
     * 应用 2D 仿射变换。
     *
     * <p>变换矩阵按列向量约定：{@code p' = M · p}，其中
     * <pre>
     *   M = [ a  c  tx ]
     *       [ b  d  ty ]
     *       [ 0  0  1  ]
     * </pre>
     * 控制点位置含平移；手柄向量只应用线性部分（a、b、c、d）。
     *
     * @param curve 曲线
     * @param a     矩阵元素 a（col0 row0）
     * @param b     矩阵元素 b（col0 row1）
     * @param c     矩阵元素 c（col1 row0）
     * @param d     矩阵元素 d（col1 row1）
     * @param tx    平移量 X
     * @param ty    平移量 Y
     * @return 变换后的新曲线
     */
    public static Curve transform(Curve curve, double a, double b, double c, double d, double tx, double ty) {
        return impl.transform(curve, a, b, c, d, tx, ty);
    }

    /**
     * 应用 2D 仿射变换（6 元素数组版本）。
     *
     * <p>数组布局：{@code [a, b, c, d, tx, ty]}，与 {@link #transform(Curve, double, double, double, double, double, double)} 参数一一对应。
     *
     * @param curve 曲线
     * @param mat2d 6 元素仿射矩阵
     * @return 变换后的新曲线
     * @throws IllegalArgumentException {@code mat2d.length != 6}
     */
    public static Curve transform(Curve curve, float[] mat2d) {
        if (mat2d.length != 6) {
            throw new IllegalArgumentException("Invalid length of mat2d!!");
        }
        return impl.transform(curve, mat2d[0], mat2d[1], mat2d[2], mat2d[3], mat2d[4], mat2d[5]);
    }

    // ═══════════════════════════════════════════════════════════
    // 偏移
    // ═══════════════════════════════════════════════════════════

    /**
     * 整条曲线沿法线方向偏移 {@code distance}。
     *
     * <p>近似算法：按段采样 → 沿法线移动 → 拟合。正距离向法线方向偏移
     * （左手系即曲线左侧），负距离反向。
     *
     * <p>偏移结果的形状是拟合近似，不是精确的等距线。
     *
     * @param curve    曲线
     * @param distance 偏移距离，可为负
     * @return 偏移后的新曲线
     */
    public static Curve offset(Curve curve, double distance) {
        return impl.offset(curve, distance);
    }

    /**
     * 段范围 {@code [fromSeg, toSeg]} 内的曲线偏移。
     *
     * <p>范围外的段原样保留。范围边界的拼接点可能出现不连续——
     * 因为偏移曲线的端点位置通常不等于原端点。调用方需自行处理边界。
     *
     * <p>返回曲线的控制点数不固定（取决于偏移结果的拟合）。
     *
     * @param curve    曲线
     * @param fromSeg  起始段索引（含）
     * @param toSeg    结束段索引（含）
     * @param distance 偏移距离，可为负
     * @return 偏移后的新曲线
     * @throws IndexOutOfBoundsException 范围非法
     */
    public static Curve offsetRange(Curve curve, int fromSeg, int toSeg, double distance) {
        return impl.offsetRange(curve, fromSeg, toSeg, distance);
    }

    // ═══════════════════════════════════════════════════════════
    // 判定
    // ═══════════════════════════════════════════════════════════

    /**
     * 判断曲线是否为一条直线段。
     *
     * <p>条件：恰有 2 个控制点、非闭合、两端手柄向量均为零。
     * 手柄阈值 {@code 1e-6}。
     *
     * @param curve 曲线
     * @return 是直线段返回 true
     */
    public static boolean isStraightLine(Curve curve) {
        List<ControlPoint> points = curve.getPoints();
        if (points.size() != 2 || curve.isClosed()) return false;
        ControlPoint p0 = points.get(0);
        ControlPoint p1 = points.get(1);
        boolean p0Straight = Math.abs(p0.getDx2()) < 1e-6 && Math.abs(p0.getDy2()) < 1e-6;
        boolean p1Straight = Math.abs(p1.getDx1()) < 1e-6 && Math.abs(p1.getDy1()) < 1e-6;
        return p0Straight && p1Straight;
    }

    // ═══════════════════════════════════════════════════════════
    // 已废弃 —— 保留兼容
    // ═══════════════════════════════════════════════════════════

    /**
     * @param curve 曲线
     * @param t     全局归一化参数
     * @return 单位切向量
     * @deprecated 用 {@link #tangent} 替代——命名简化。
     */
    @Deprecated
    public static Pair unitTangent(Curve curve, double t) {
        return impl.tangent(curve, t);
    }

    /**
     * @param curve 曲线
     * @param t     全局归一化参数
     * @return 单位法向量
     * @deprecated 用 {@link #normal} 替代——命名简化。
     */
    @Deprecated
    public static Pair unitNormal(Curve curve, double t) {
        return impl.normal(curve, t);
    }

    /**
     * 焊接两个相邻控制点。
     *
     * <p>位置和连续性取自 {@code idxActive}；入切取自两点中的前点，
     * 出切取自后点。两点之间的手柄（前点的出切、后点的入切）消失。
     *
     * <p>要求 {@code |idxActive - idxPassive| == 1}。焊接后控制点数 -1。
     *
     * @param curve      原曲线
     * @param idxActive  主动点索引（位置、连续性以它为准）
     * @param idxPassive 被动点索引（被合并掉）
     * @return 焊接后的新曲线
     */
    public static Curve weldAdjacent(Curve curve, int idxActive, int idxPassive) {
        return impl.weldAdjacent(curve, idxActive, idxPassive);
    }

    /**
     * 焊接两条曲线的端点，合并为一条曲线。
     *
     * <p>调用方需保证 left 的末点和 right 的首点是被 weld 的两个点。
     * 入切取左末点的入切，出切取右首点的出切；位置和连续性取自 active。
     *
     * @param left                  左曲线
     * @param right                 右曲线
     * @param activeIsLeftEndpoint  true = active 是左末点；false = active 是右首点
     * @return 焊接后的新曲线，非闭合
     */
    public static Curve weldJoin(Curve left, Curve right, boolean activeIsLeftEndpoint) {
        return impl.weldJoin(left, right, activeIsLeftEndpoint);
    }

    // ═══════════════════════════════════════════════════════════
    // Spec 接口
    // ═══════════════════════════════════════════════════════════

    /**
     * 曲线操作的策略接口。
     *
     * <p>{@link Bezier2D} 的每个静态方法都委托给本接口的默认实现
     * {@link Bezier2DImpl}。接口本身不提供文档——具体语义见
     * {@link Bezier2D} 上对应的静态方法。
     *
     * <p><b>实现者约定：</b>实现类不得改变任何方法的语义（返回值、异常、
     * 边界行为）。本接口仅用于替换实现（如使用不同的拟合算法、加速结构），
     * 不是为了扩展 API。
     */
    public interface Spec {

        // ─── 求值 ────────────────────────────────────────────

        Pair eval(Curve curve, double t);
        Pair deriv(Curve curve, double t);
        Pair deriv2(Curve curve, double t);
        Pair tangent(Curve curve, double t);
        Pair normal(Curve curve, double t);
        double curvature(Curve curve, double t);
        Pair[] sample(Curve curve, int count);

        // ─── 最近点 ──────────────────────────────────────────

        ClosestPointResult closestPoint(Curve curve, Pair point);
        ClosestPointResult closestPointRange(Curve curve, Pair point, int fromSeg, int toSeg);

        // ─── AABB ────────────────────────────────────────────

        AABB aabb(Curve curve);

        /** @deprecated 用 {@link #aabbRange} 替代 */
        @Deprecated
        AABB aabb(Curve curve, int idx);

        AABB aabbRange(Curve curve, int fromSeg, int toSeg);

        // ─── 拓扑编辑 ────────────────────────────────────────

        Curve insertPoint(Curve curve, double t);
        Curve insertPoint(Curve curve, int segIdx, double localT);

        Curve deletePoint(Curve curve, int idx);

        Curve reverse(Curve curve);

        Curve join(Curve left, Curve right);

        void cut(Curve curve, double t, Curve out1, Curve out2);
        void split(Curve curve, int idx, Curve out1, Curve out2);

        // ─── 重构 / 拟合 ─────────────────────────────────────

        Curve reform(Curve curve, int count);
        Curve reformRange(Curve curve, int fromSeg, int toSeg, int newSegCount);

        Curve fit(double[] xs, double[] ys, double maxError, int maxSeg);

        // ─── 变换 ────────────────────────────────────────────

        Curve translate(Curve curve, double dx, double dy);
        Curve scale(Curve curve, double sx, double sy, double cx, double cy);
        Curve transform(Curve curve, double a, double b, double c, double d, double tx, double ty);

        // ─── 偏移 ────────────────────────────────────────────

        Curve offset(Curve curve, double distance);
        Curve offsetRange(Curve curve, int fromSeg, int toSeg, double distance);


        Curve weldAdjacent(Curve curve, int idxActive, int idxPassive);
        Curve weldJoin(Curve left, Curve right, boolean activeIsLeftEndpoint);

    }
}