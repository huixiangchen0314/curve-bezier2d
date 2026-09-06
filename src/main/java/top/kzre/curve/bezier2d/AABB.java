package top.kzre.curve.bezier2d;

import lombok.*;

@Builder
@ToString
@Value
public class AABB {
    double minX;
    double minY;
    double maxX;
    double maxY;

    // 自定义构造函数，加入约束
    public AABB(double minX, double minY, double maxX, double maxY) {
        // 检查 NaN
        if (Double.isNaN(minX) || Double.isNaN(minY) ||
                Double.isNaN(maxX) || Double.isNaN(maxY)) {
            throw new IllegalArgumentException("Coordinates must not be NaN");
        }
        // 检查 min <= max
        if (minX > maxX) {
            throw new IllegalArgumentException("minX (" + minX + ") must be <= maxX (" + maxX + ")");
        }
        if (minY > maxY) {
            throw new IllegalArgumentException("minY (" + minY + ") must be <= maxY (" + maxY + ")");
        }

        this.minX = minX;
        this.minY = minY;
        this.maxX = maxX;
        this.maxY = maxY;
    }

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
