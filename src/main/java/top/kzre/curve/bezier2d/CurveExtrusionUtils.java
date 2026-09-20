package top.kzre.curve.bezier2d;

import java.util.ArrayList;
import java.util.List;


@Deprecated
public final class CurveExtrusionUtils {

    private CurveExtrusionUtils() {}

    /**
     * 在曲线头部挤出新点，返回更新后的 t 参数数组。
     * @param oldCurve    原始曲线
     * @param oldTParams  原始 t 参数数组（递增，范围 [0,1]）
     * @param newPoint    新插入的起点
     * @param out         用于输出新曲线的容器（将修改其控制点列表）
     * @return 新的 t 参数数组（长度 = oldTParams.length + 1）
     */
    public static double[] extrudeHead(Curve oldCurve, double[] oldTParams,
                                       Pair newPoint, Curve out) {
        if (oldCurve.isClosed()) {
            throw new IllegalArgumentException("Cannot extrude closed curve");
        }
        int oldSegmentCount = oldCurve.getSegmentCount();
        int newSegmentCount = oldSegmentCount + 1;

        double[] newTParams = new double[oldTParams.length + 1];
        newTParams[0] = 0.0;
        for (int i = 1; i < oldTParams.length; i++) {
            // t_new = (t_old * oldSegmentCount + 1) / newSegmentCount
            newTParams[i] = (oldTParams[i] * oldSegmentCount + 1) / newSegmentCount;
        }

        // 构建新曲线（头部插入新点）
        List<ControlPoint> newPoints = new ArrayList<>();
        // 新起点（手柄为零）
        newPoints.add(ControlPoint.builder()
                .x(newPoint.getX())
                .y(newPoint.getY())
                .build());
        // 旧控制点（保留原手柄）
        newPoints.addAll(oldCurve.getPoints());
        out.setPoints(newPoints);
        out.setClosed(false);

        return newTParams;
    }

    /**
     * 在曲线尾部挤出新点，返回更新后的 t 参数数组。
     * @param oldCurve    原始曲线
     * @param oldTParams  原始 t 参数数组（递增，范围 [0,1]）
     * @param newPoint    新插入的终点
     * @param out         用于输出新曲线的容器（将修改其控制点列表）
     * @return 新的 t 参数数组（长度 = oldTParams.length + 1）
     */
    public static double[] extrudeTail(Curve oldCurve, double[] oldTParams,
                                       Pair newPoint, Curve out) {
        if (oldCurve.isClosed()) {
            throw new IllegalArgumentException("Cannot extrude closed curve");
        }
        int oldSegmentCount = oldCurve.getSegmentCount();
        int newSegmentCount = oldSegmentCount + 1;

        double[] newTParams = new double[oldTParams.length + 1];
        for (int i = 0; i < oldTParams.length; i++) {
            // t_new = t_old * oldSegmentCount / newSegmentCount
            newTParams[i] = oldTParams[i] * oldSegmentCount / newSegmentCount;
        }
        newTParams[oldTParams.length] = 1.0;

        // 构建新曲线（尾部插入新点）
        List<ControlPoint> newPoints = new ArrayList<>(oldCurve.getPoints());
        // 新终点（手柄为零）
        newPoints.add(ControlPoint.builder()
                .x(newPoint.getX())
                .y(newPoint.getY())
                .build());
        out.setPoints(newPoints);
        out.setClosed(false);

        return newTParams;
    }
}