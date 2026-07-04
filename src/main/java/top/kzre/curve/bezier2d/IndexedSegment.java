package top.kzre.curve.bezier2d;

import lombok.AllArgsConstructor;
import lombok.ToString;
import lombok.Value;

@ToString
@AllArgsConstructor
@Value
public class IndexedSegment {
    Segment segment;
    double local;
    int index;
}
