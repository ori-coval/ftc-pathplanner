package com.pathplanner.lib.missingWpilibClasses.math;

import com.pathplanner.lib.missingWpilibClasses.math.numbers.N1;
import com.pathplanner.lib.missingWpilibClasses.math.numbers.N2;

import org.ejml.simple.SimpleMatrix;

import java.util.Objects;

/** A class for constructing vectors (Nx1 matrices). */
public final class VecBuilder {
    private VecBuilder() {
        throw new UnsupportedOperationException("this is a utility class!");
    }

    private static <N extends Num> Vector<N> fillVec(Nat<N> rows, double... data) {
        if (Objects.requireNonNull(data).length != rows.getNum()) {
            throw new IllegalArgumentException(
                    "Invalid vector data provided. Wanted "
                            + rows.getNum()
                            + " vector, but got "
                            + data.length
                            + " elements");
        }
        return new Vector<>(new SimpleMatrix(new double[][]{data}));
    }

    /**
     * Returns a 1x1 vector containing the given elements.
     *
     * @param n1 the first element.
     * @return 1x1 vector
     */
    public static Vector<N1> fill(double n1) {
        return fillVec(Nat.N1(), n1);
    }

    /**
     * Returns a 2x1 vector containing the given elements.
     *
     * @param n1 the first element.
     * @param n2 the second element.
     * @return 2x1 vector
     */
    public static Vector<N2> fill(double n1, double n2) {
        return fillVec(Nat.N2(), n1, n2);
    }

}
