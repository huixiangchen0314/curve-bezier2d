package top.kzre.curve.bezier2d;

import lombok.*;

@Builder
@AllArgsConstructor
@ToString
@Value
public class AABB {
    double minX;
    double minY;
    double maxX;
    double maxY;

    /**
     * 合并另一个包围盒，返回能同时容纳两者的最小包围盒。
     */
    public AABB merge(AABB other) {
        if (other == null) return this;
        return new AABB(
                Math.min(this.minX, other.minX),
                Math.min(this.minY, other.minY),
                Math.max(this.maxX, other.maxX),
                Math.max(this.maxY, other.maxY)
        );
    }
    /** 宽 */
    public double width() {
        return maxX - minX;
    }

    /** 高 */
    public double height() {
        return maxY - minY;
    }


}
