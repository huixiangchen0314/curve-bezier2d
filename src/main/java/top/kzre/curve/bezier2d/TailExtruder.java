package top.kzre.curve.bezier2d;

import java.util.ArrayList;
import java.util.List;

/**
 * 尾部挤出器：在曲线终点插入新点，并更新参数映射表。
 */
public abstract class TailExtruder implements TableMappingCurveExtruder {


    @Override
    public TableMapping extrude(Curve curve, TableMapping table, Pair point, Curve out) {
        if (curve.isClosed()) {
            throw new IllegalArgumentException("curve is closed");
        }

        double[] oldTParams = table.getTParams();
        int oldSegmentCount = curve.getSegmentCount();          // 旧曲线段数
        int newSegmentCount = oldSegmentCount + 1;              // 新曲线段数

        // 新参数表长度 = 旧参数长度 + 1（新终点）
        double[] newTParams = new double[oldTParams.length + 1];
        // 旧参数映射到新曲线
        for (int i = 0; i < oldTParams.length; i++) {
            // t_new = t_old * oldSegmentCount / newSegmentCount
            newTParams[i] = oldTParams[i] * oldSegmentCount / newSegmentCount;
        }
        newTParams[oldTParams.length] = 1.0;  // 新终点参数为1

        // 构建新控制点列表
        List<ControlPoint> newPoints = new ArrayList<>(curve.getPoints());
        // 追加新点
        newPoints.add(ControlPoint.builder()
                .x(point.getX())
                .y(point.getY())
                .build());
        out.setPoints(newPoints);

        // 抽象方法：由子类实现（弦长或弧长）
        double[] newSParams = buildSParams(out, newTParams);
        return new TableMapping(newTParams, newSParams);
    }

    /**
     * 根据曲线和 t 参数列表，计算对应的 s 参数（累积长度或弦长）。
     * 由子类实现具体的映射类型。
     */
    protected abstract double[] buildSParams(Curve curve, double[] tParams);
}