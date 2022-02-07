package org.vulcanrobotics.math.utils;

public class Utils {
    public static double clip(double num, double min, double max){
        if(num < min){
            num = min;
        }
        if(num > max){
            num = max;
        }
        return num;
    }

}
