package org.firstinspires.ftc.teamcode.Utils;

public class MathAccessories {
    // make all of the array values absolute
    public static double[] arrayAbs(double[] arr){
        for (int i = 0; i < arr.length; i++)
            arr[i] = Math.abs(arr[i]);

        return arr;
    }

    // get the biggest value from the array
    public static double arrayMax(double[] arr){
        double max = arr[0];

        for (int i = 1; i < arr.length; i++)
            max = Math.max(max, arr[i]);

        return max;
    }

    // make sure all of the array values relativity is kept and that the largest value is 1 or lower
    public static double[] normalize(double[] arr){
        double biggestNum = arrayMax(arrayAbs(arr));

        if (biggestNum > 1) {
            for (int i = 0; i < arr.length; i++) {
                arr[i] /= biggestNum;
            }
        }
        return arr;
    }
}