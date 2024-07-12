package org.firstinspires.ftc.teamcode.MMLib.Utils;

import static java.util.Objects.requireNonNull;

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
     *
     * @param x      value
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

    /**
     * Requires that a parameter of a method not be null; prints an error message with helpful
     * debugging instructions if the parameter is null.
     *
     * @param <T>        Type of object.
     * @param obj        The parameter that must not be null.
     * @param paramName  The name of the parameter.
     * @param methodName The name of the method.
     * @return The object parameter confirmed not to be null.
     */
    public static <T> T requireNonNullParam(T obj, String paramName, String methodName) {
        return requireNonNull(
                obj,
                "Parameter "
                        + paramName
                        + " in method "
                        + methodName
                        + " was null when it"
                        + " should not have been!  Check the stacktrace to find the responsible line of code - "
                        + "usually, it is the first line of user-written code indicated in the stacktrace.  "
                        + "Make sure all objects passed to the method in question were properly initialized -"
                        + " note that this may not be obvious if it is being called under "
                        + "dynamically-changing conditions!  Please do not seek additional technical assistance"
                        + " without doing this first!"
        );
    }


}
