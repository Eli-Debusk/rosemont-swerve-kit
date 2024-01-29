package frc.montylib;

public class MontyMath {

    /**
     * Restricts a given value to in between two limits
     * @param value the value to limit
     * @param min the lower limit
     * @param max the upper limit
     * @return a value between the lower and upper limit
     */
    public static double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    /**
     * Discards a given value if its absolute value is below a given threshold
     * @param value the value to discard or return
     * @param threshold the value threshold which consititutes at which point the value is discarded
     * @return either 0.0 (if the value is below the threshold) or the given value (if the value is above the threshold)
     */
    public static double applyValueThreshold(double value, double threshold) {
        return Math.abs(value) > threshold ? value : 0;
    }

    /**
     * Returns the difference between two values   
     * @implNote you can input numbers in any order
     * @param x the first value to use
     * @param y the second value to use
     * @return the difference of the two values
     */
    public static double valueDifference(double x, double y) {
        return x > y ? x - y : y - x;
    }
}
