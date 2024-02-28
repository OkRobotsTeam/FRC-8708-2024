package frc.robot;

import static java.lang.Math.pow;

public class MathUtils {
    public static double cubicFilter(double input, double linearity) {
        return pow(input, 3) * (1 - linearity) + input * linearity;
    }
}
