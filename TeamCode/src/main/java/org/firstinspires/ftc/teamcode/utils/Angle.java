package org.firstinspires.ftc.teamcode.utils;

public class Angle {
    public static double norm(double a) { // wraps angle to be between 0 and 2pi
        if(a >= 0 && a < Math.PI * 2)
            return a;
        double rev = Math.PI * 2;
        return ((a % (rev)) + rev) % rev;
    }
    public static double normDelta(double a) { // wraps angle to be between -pi and pi
        a = norm(a);
        if(a > Math.PI)
            a -= Math.PI * 2;
        return a;
    }
}