package frc.robot;

import java.util.HashMap;
import java.util.Map;
public class Debug {
    private static int i = 0;
    private static int[] ids = { 0,0,0,0,0,0,0,0,0};
    private static Map<String, Integer> map = new HashMap<String, Integer>();

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
