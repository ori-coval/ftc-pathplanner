package org.firstinspires.ftc.teamcode.MMLib;

import java.util.function.DoubleSupplier;

/**
 * this class represents useful method that may help you
 */
public abstract class MMUtils {


    /**
     * this method represents f(x),
     * while f is the linear function created by the 2 points.
     * in order to calculate the slope of the line, the method requires 2 points.
     * usually it's easier to think about the starting point, and the ending point,
     * but any 2 points will work.
     * @param x value
     * @param point1 any point on graph
     * @param point2 any point on graph
     * @return the value of x in the linear function (f(x))
     */
    public static double mapValuesLinear(double x, MMPoint2D point1, MMPoint2D point2) {
        double m = (point2.y - point1.y) / (point2.x - point1.x);
        return m * (x - point1.x) + point1.y;
    }

    public static double mapValuesLinear(DoubleSupplier x, MMPoint2D point1, MMPoint2D point2) {
        return mapValuesLinear(x.getAsDouble(), point1, point2);
    }

    public static double joystickToServo(double joystick) {
        return mapValuesLinear(
                joystick,
                new MMPoint2D(-1, 0),
                new MMPoint2D(1, 1)
        );
    }


}
