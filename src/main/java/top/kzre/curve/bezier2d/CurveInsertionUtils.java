package top.kzre.curve.bezier2d;

public class CurveInsertionUtils {

    /**
     * 在 全局标准参数t 插入 点，并更新 t 参数
     */
    public static double[] insertAtPoint(Curve oldCurve, double[] oldTParams,
                                         int t, Curve out) {

        Curve curve = Bezier2D.insertPoint(oldCurve, t);
        out.setPoints(curve.getPoints());
        out.setClosed(curve.isClosed());
        return null;
    }
}
