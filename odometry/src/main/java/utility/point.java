package utility;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

//XXX: please use java naming conventions. They exist for a reason
public class point {
    public double x, y;

    public point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public static point rotate(double x, double y, double r) {
        return new point(
                x * cos(r) - y * sin(r),
                y * cos(r) + x * sin(r));
    }

    public point rotate(double r) {
        return rotate(x, y, r);
    }
}
