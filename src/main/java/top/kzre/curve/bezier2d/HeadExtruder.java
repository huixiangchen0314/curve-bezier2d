package top.kzre.curve.bezier2d;

import java.util.ArrayList;
import java.util.List;

/**
 * 弦长表，从头部挤出点。
 */
public abstract class HeadExtruder implements TableMappingCurveExtruder{

    /**
     * 关键是计算 补充来的 t
     * 先构建新曲线
     * 计算旧tParams 在新 曲线上的值重新构建 tParams
     * 根据新tParams 在新曲线上获取数值，重新构建 sParams
     */
    @Override
    public TableMapping extrude(Curve curve, TableMapping table, Pair point, Curve out) {
        if(curve.isClosed()){
            throw new IllegalArgumentException("curve is closed");
        }

        double[] tParams = table.getTParams();
        double[] newTParams = new double[tParams.length + 1];
        int segmentCount = curve.getSegmentCount();
        // 头部 采样点 t 参数保持为0
        newTParams[0] = 0;
        // 重新计算 t 参数
        for (int i = 1; i < tParams.length; i++) {
            newTParams[i] = (tParams[i] * segmentCount + 1) / segmentCount + 1;
        }
        // 构建新曲线
        List<ControlPoint> newPoints = new ArrayList<>();
        // 新锚点
        newPoints.add(ControlPoint.builder()
                .x(point.getX())
                .y(point.getY())
                .build());
        // 旧锚点
        newPoints.addAll(curve.getPoints());
        out.setPoints(newPoints);

        // 抽象函数, 重新构建新s参数
        double[] sParams = buildSParams(out, newTParams);
        return new TableMapping(newTParams, sParams);
    }

    protected abstract double[] buildSParams(Curve curve, double[] tParams);
}
