package frc.robot.config;

public class ClimbConstants {
    // Specs
    public static final double maxVolts = 12.0;
    public static final int currentLimit = 60; 
    public static final double latchDelay_s = 0.5;   

    // Physics
    public static final double dt_s = 0.02; // seconds

    // Feedforward
    public static final double ks_V = 0; 
    public static final double kv_Vm_r = 0; 
    public static final double ka_Vm2_r = 0;

    
    // Control (winch is positive)
    public static final double unspoolVolts = 12.0;
    public static final double winchVolts = -12.0;
    public static final double stallVolts = -12.0;
    public static final double stallCurrent = 40.0;


    public static final double unspoolDistance = 16057.0;
    public static final double winchDistance = -23723.0;
    public static final double tugDistance = -4000.0;
    public static final double stallDistance = -40.0;


    public static final double kPu = 0.2;
    public static final double kIu = 0.0;
    public static final double kDu = 0.0;
    public static final double kFu = 0.0;

    public static final double kPw = 0.0;
    public static final double kIw = 0.0;
    public static final double kDw = 0.0;
    public static final double kFw = 0.0;

    public static final double cruiseVel = 80000.0;
    public static final double acc = 2500.0;
    public static final int smoothing = 1; // set from 0-8

}
