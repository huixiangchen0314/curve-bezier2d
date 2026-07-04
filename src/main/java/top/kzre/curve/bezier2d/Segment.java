package top.kzre.curve.bezier2d;

import lombok.*;

@AllArgsConstructor
@ToString
@Getter
@Builder
public class Segment {
    private Pair a;  // P0
    private Pair b;  // P1
    private Pair c;  // P2
    private Pair d;  // P3

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

    public Segment setA(Pair a) { this.a = a; return this; }
    public Segment setB(Pair b) { this.b = b; return this; }
    public Segment setC(Pair c) { this.c = c; return this; }
    public Segment setD(Pair d) { this.d = d; return this; }
    public Segment set(Segment s) { this.a = s.a; this.b = s.b; this.c = s.c; this.d = s.d; return this; }
    public Segment set(Pair a, Pair b, Pair c, Pair d) { this.a = a; this.b = b; this.c = c; this.d = d; return this; }
}