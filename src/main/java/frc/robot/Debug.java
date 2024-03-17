package frc.robot;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.HashMap;
import java.util.Map;


public class Debug {
    private static int i = 0;
    private static final int[] ids = {0,0,0,0,0,0,0,0,0};
    private static final Map<String, Integer> map = new HashMap<>();
    public static NumberFormat fourPlaces = new DecimalFormat("0.0000");
    public static NumberFormat sixPlaces = new DecimalFormat("0.000000");
    public static NumberFormat twoPlaces = new DecimalFormat("0.00");
    

    
    public static void debugPrint(String s) {
        if (i++%10==0) {
            System.out.println(s);
        }
    }
    public static void debugPrint(int id, String s) {
        if (ids[id]++%10==0) {
            System.out.println("id " + id + ": " +  s);
        }
    }


     public static String twoPlaces(double number) {
        return twoPlaces.format(number);
    }
    public static String fourPlaces(double number) {
        return fourPlaces.format(number);
    }
    public static String sixPlaces(double number) {
        return sixPlaces.format(number);
    }

    public static void print(String key, String s) {
        debugPrint(key, s);

    }
    public static void debugPrint(String key, String s) {
        Integer i = map.get(key);
        if (i == null ) { 
            i=0;
        }
        if (i++%10==0) {
            System.out.println(i+" " + key+ ": " + s);
        }
        map.put(key, i);
        
    }
}
