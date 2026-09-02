package top.kzre.curve.bezier2d;

import lombok.*;


@AllArgsConstructor
@ToString
@Getter
@Builder
public final class Segment {
    private Pair a;  // P0
    private Pair b;  // P1
    private Pair c;  // P2
    private Pair d;  // P3

    Segment(){

    }

    public static Segment of(ControlPoint a, ControlPoint b) {
        double x0 = a.getX(), y0 = a.getY();
        double x3 = b.getX(), y3 = b.getY();

        double dxOut = a.getDx2(), dyOut = a.getDy2();
        double p1x, p1y;
        if (Math.abs(dxOut) < 1e-12 && Math.abs(dyOut) < 1e-12) {
            p1x = x0 + (x3 - x0) / 3.0;
            p1y = y0 + (y3 - y0) / 3.0;
        } else {
            p1x = x0 + dxOut;
            p1y = y0 + dyOut;
        }

        double dxIn = b.getDx1(), dyIn = b.getDy1();
        double p2x, p2y;
        if (Math.abs(dxIn) < 1e-12 && Math.abs(dyIn) < 1e-12) {
            p2x = x0 + 2.0 * (x3 - x0) / 3.0;
            p2y = y0 + 2.0 * (y3 - y0) / 3.0;
        } else {
            p2x = x3 + dxIn;   // 注意：加号
            p2y = y3 + dyIn;
        }

        return new Segment(
                new Pair(x0, y0),
                new Pair(p1x, p1y),
                new Pair(p2x, p2y),
                new Pair(x3, y3)
        );
    }

    /**
     * 该方法仅仅用于多返回结果。包内部保证外部不可变性。
     */
    Segment set(Pair a, Pair b, Pair c, Pair d) {
        this.a = a;
        this.b = b;
        this.c = c;
        this.d = d;
        return this;
    }



    /**
     * 将段的数据写入两个控制点（锚点）。
     * 更新 p1 的坐标和出切线，p2 的坐标和入切线。
     */
    public void write(ControlPoint p1, ControlPoint p2) {
        // 起点锚点
        p1.setX(a.getX());
        p1.setY(a.getY());
        p1.setDx2(b.getX() - a.getX());  // 出切线 = P1 - P0
        p1.setDy2(b.getY() - a.getY());

        // 终点锚点
        p2.setX(d.getX());
        p2.setY(d.getY());
        p2.setDx1(c.getX() - d.getX());  // 入切线 = P2 - P3
        p2.setDy1(c.getY() - d.getY());

    }
}