package top.kzre.curve.bezier2d;

import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.Value;

@Getter
@AllArgsConstructor
@Value
public class ClosestPointResult {
    Pair point;
    double t;
    double distance;
}
