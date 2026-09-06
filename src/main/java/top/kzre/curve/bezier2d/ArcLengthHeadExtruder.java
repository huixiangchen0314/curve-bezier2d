package top.kzre.curve.bezier2d;

public final class ArcLengthHeadExtruder extends HeadExtruder{
    public static final ArcLengthHeadExtruder INSTANCE = new ArcLengthHeadExtruder();
    private ArcLengthHeadExtruder() {}

    @Override
    protected double[] buildSParams(Curve curve, double[] tParams) {
        return ArcLengthUtils.buildArcLengthParams(curve, tParams);
    }
}
