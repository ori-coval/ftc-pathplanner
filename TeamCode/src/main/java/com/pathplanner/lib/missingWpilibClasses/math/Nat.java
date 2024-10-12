package com.pathplanner.lib.missingWpilibClasses.math;

// CHECKSTYLE.OFF: ImportOrder

// CHECKSTYLE.ON

import com.pathplanner.lib.missingWpilibClasses.math.numbers.N0;
import com.pathplanner.lib.missingWpilibClasses.math.numbers.N1;
import com.pathplanner.lib.missingWpilibClasses.math.numbers.N2;

/**
 * A natural number expressed as a java class.
 * The counterpart to {@link Num} that should be used as a concrete value.
 *
 * @param <T> The {@link Num} this represents.
 */
public interface Nat<T extends Num> {
    /**
     * The number this interface represents.
     *
     * @return The number backing this value.
     */
    int getNum();

    /**
     * Returns the Nat instance for 0.
     *
     * @return The Nat instance for 0.
     */
    static Nat<N0> N0() {
        return N0.instance;
    }

    /**
     * Returns the Nat instance for 1.
     *
     * @return The Nat instance for 1.
     */
    static Nat<N1> N1() {
        return N1.instance;
    }

    /**
     * Returns the Nat instance for 2.
     *
     * @return The Nat instance for 2.
     */
    static Nat<N2> N2() {
        return N2.instance;
    }
}
