package frc.robot;

import static java.lang.Math.pow;

public class MathUtils {
    public static double cubicFilter(double input, double linearity) {
        return pow(input, 3) * (1 - linearity) + input * linearity;
    }
    
    public static int clamp(int value, int min, int max) {
        return Math.max(min, Math.min(max, value));
    }  

    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }  
}
