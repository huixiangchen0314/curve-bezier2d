package top.kzre.curve.bezier2d;

import lombok.*;

@NoArgsConstructor
@AllArgsConstructor
@Builder
@ToString
@Getter
public final class ControlPoint {
    private double x;
    private double y;
    private double dx1;   // 入切向量：从该点指向前一个内部控制点
    private double dy1;
    private double dx2;   // 出切向量：从该点指向后一个内部控制点
    private double dy2;
    private boolean g1;

     ControlPoint setX(double x) { this.x = x; return this; }
     ControlPoint setY(double y) { this.y = y; return this; }
     ControlPoint setDx1(double dx1) { this.dx1 = dx1; return this; }
     ControlPoint setDy1(double dy1) { this.dy1 = dy1; return this; }
     ControlPoint setDx2(double dx2) { this.dx2 = dx2; return this; }
     ControlPoint setDy2(double dy2) { this.dy2 = dy2; return this; }
     ControlPoint setG1(boolean g1) { this.g1 = g1; return this; }

    public Pair toPosition() { return new Pair(x, y); }
    public Pair toTangentIn() { return new Pair(dx1, dy1); }
    public Pair toTangentOut() { return new Pair(dx2, dy2); }

    public void applyConstraints() {
        if (g1) {
            double lenIn = Math.hypot(dx1, dy1);
            double lenOut = Math.hypot(dx2, dy2);
            if (lenOut < 1e-12) { dx1 = 0; dy1 = 0; return; }
            if (lenIn < 1e-12) {
                dx1 = -dx2;
                dy1 = -dy2;
            } else {
                double factor = lenIn / lenOut;
                dx1 = -dx2 * factor;
                dy1 = -dy2 * factor;
            }
        }
    }

    public ControlPoint copy() {
        return new ControlPoint(x, y, dx1, dy1, dx2, dy2, g1);
    }
}