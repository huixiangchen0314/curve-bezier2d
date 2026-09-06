package top.kzre.curve.bezier2d;

public final class ArcLengthTailExtruder extends TailExtruder {

    public static final ArcLengthTailExtruder INSTANCE = new ArcLengthTailExtruder();
    private ArcLengthTailExtruder() {}

    @Override
    protected double[] buildSParams(Curve curve, double[] tParams) {
        return ArcLengthUtils.buildArcLengthParams(curve, tParams);
    }
}
