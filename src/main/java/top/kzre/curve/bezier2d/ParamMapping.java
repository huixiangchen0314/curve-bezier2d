package top.kzre.curve.bezier2d;

/**
 * 参数映射接口
 */
public interface ParamMapping {

    /**
     * 标准参数 t 转 映射参数 s
     */
    double getS(double t);

    /**
     * 映射参数 s 转标准参数 t
     */
    double getT(double s);
}
