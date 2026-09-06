package top.kzre.curve.bezier2d;


/**
 * 用于在曲线挤出操作时更新参数映射表。
 * 实现类应针对特定的映射类型（如弦长、弧长）提供更新逻辑。
 */
@FunctionalInterface
public interface TableMappingCurveExtruder {
    /**
     * 在曲线中插入一个新点（挤出），返回更新后的参数映射表。
     *
     * @param curve     原始曲线
     * @param table     旧映射表（t ↔ s）
     * @param point     插入的新点（通常是端点挤出，但也可以是中间插入）
     * @return 更新后的映射表
     */
    TableMapping extrude(Curve curve, TableMapping table, Pair point, Curve out);

    static TableMapping extrudeArcLengthHead(Curve curve, TableMapping table, Pair point, Curve out) {
       return ArcLengthHeadExtruder.INSTANCE.extrude(curve, table, point, out);
    }

    static TableMapping extrudeArcLengthTail(Curve curve, TableMapping table, Pair point, Curve out) {
         return ArcLengthTailExtruder.INSTANCE.extrude(curve, table, point, out);
    }
}