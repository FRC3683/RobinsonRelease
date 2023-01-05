/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

/**
 * Add your docs here.
 */
public class MathUtils {
    public static double sq(double x){
        return x * x;
    }
    public static double average(double a, double b) {
        return (a+b)/2;
    }
    public static double cube(double x){
        return x * x * x;
    }
    public static double lerp(double start, double end, double alpha){
        return start * (1.0 - alpha) + end * alpha;
    }
    public static double unlerp(double start, double end, double value){
        return (value - start) / (end - start);
    }
    public static double remap(double old_start, double old_end, double old_value, double new_start, double new_end){
        return lerp(new_start, new_end, unlerp(old_start, old_end, old_value));
    }
    public static double cubicBezier(double start, double start_q, double end, double end_q, double alpha){
        double a = start * cube(1.0 - alpha);
        double b = 3 * start_q * sq(1.0 - alpha) * alpha;
        double c = 3 * end_q * (1.0 - alpha) * sq(alpha);
        double d = end * cube(alpha);
        return a + b + c + d;
    }
    public static double clamp(double min, double max, double value){
        return value < min ? min : (value > max ? max : value);
    }
    public static double clamp01(double value){
        return clamp(0.0, 1.0, value);
    }
    public static double clamp1(double value){
        return clamp(-1.0, 1.0, value);
    }
    public static double clampN(double n, double value){
        return clamp(-n, n, value);
    }
    public static double clamp0N(double max, double value){
        return clamp(0.0, max, value);
    }
    public static double squaredInput(double x) {
        return x > 0 ? sq(x) : -sq(x);
    }
    public static double calcLeftDrive(double thrust, double rotate){
        return -thrust - rotate;
    }
    public static double calcRightDrive(double thrust, double rotate){
        return -thrust + rotate;
    }

    public static double calcLeftDriveDynamic(double thrust, double rotate){
        double result = 0.0;
        if(thrust > 0.0){
            if(rotate > 0.0f){
                result = -sq(thrust) - sq(rotate);
            } else{
                result = -sq(thrust) + sq(rotate);
            }
        }else{
            if(rotate > 0.0f){
                result = sq(thrust) - sq(rotate);
            } else{
                result = sq(thrust) + sq(rotate);
            }
        }
        return result;
    }

    public static double calcRightDriveDynamic(double thrust, double rotate){
        double result = 0.0;
        if(thrust > 0.0){
            if(rotate > 0.0f){
                result = -sq(thrust) + sq(rotate);
            } else{
                result = -sq(thrust) - sq(rotate);
            }
        }else{
            if(rotate > 0.0f){
                result = sq(thrust) + sq(rotate);
            } else{
                result = sq(thrust) - sq(rotate);
            }
        }
        return result;
    }

    public static boolean closeEnough(double num1, double num2, double error) {
        return Math.abs(num1-num2) <= error;
    }

    public static boolean closeEnoughPercent(double num1, double num2, double alpha) {
        return Math.abs(num1-num2) <= Math.abs(num2 * alpha);
    }

}