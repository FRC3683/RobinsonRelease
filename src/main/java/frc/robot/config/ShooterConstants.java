package frc.robot.config;

public class ShooterConstants {
    public static final String shotmapFile = "shotmap.isaac";
    public static final String farshotmapFile = "farshotmap.isaac";
    public static final String speedmapFile = "speedmap.isaac";
    public static final String accmapFile = "accmap.isaac";
    
    // RPMs
    public static final double rpm_prerev = 1000;
    public static final double rpm_freethrow = 1800;
    public static final double rpm_ideal = 1935;
    public static final double rpm_far = 2300;
    public static final double rpm_max = 5000;
    public static final double rpm_rev = 1750;
    public static final double rpm_reject = 500;

    // Specs
    public static final double maxVolts = 12.0;
    public static final int currentLimit = 60;    
    public static final boolean enableSpindown = false;    
    public static final double timeToFullOpen = 0.0;
    public static final double timeToFullClosed = 0.0; 
    public static final double chamberDelay_s = 0.1;    
    public static final double hoodUpDist = 225.0;//210.0; //was 194.0 before danny changed on 2022-03-17   
    public static final double hoodHysteresis = 1.0;    
    public static final double hoodDebouncerTime = 0.1;    
    public static final double auxRPMRatio = 4.0;    

    // Physics
    public static final double dt_s = 0.02; // seconds
    public static final double momentOfInertia_Kgm2 = 0.0013; // Kg*m^2
    public static final double gearRatio = 1.0;
    public static final double launchAngle_deg = 66.0; // degrees

    // Feedforward
    public static final double ks_V = 0.1088; 
    public static final double kv_Vm_r = 0.12255; 
    public static final double ka_Vm2_r = 0.0056509;
    
    public static final double aux_ks_V = 0.13305; 
    public static final double aux_kv_Vm_r = 0.064026; 
    public static final double aux_ka_Vm2_r = 0.018431;

    // Feedback
    public static final double ffScale = 1.2;
    public static final double fastRampToPreciseThreshold = 0.97;
    public static final double rpmDenominator = 60.0;
    public static final double cpr = 1440.0;
    public static final double decayRate = 0.78;
    public static final double prerevTime_s = 2.0;
    public static final long prerevSteps = Math.round(prerevTime_s / dt_s);

//Main wheels
    //SPINUP
    public static final double kP0 = 0; //cpr * 1.827e-7;
    public static final double kI0 = 0;
    public static final double kD0 = 0;
    public static final double kFF0 = ffScale * kv_Vm_r/12.0/ShooterConstants.rpmDenominator;
    public static final double kIz0 = 0;
    public static final double kMinOutput0 = 0.0;
    public static final double kMaxOutput0 = 1.0;
    
    //SHOOTING
    public static final double kP1 = cpr * 1.227e-7;
    public static final double kI1 = cpr * 5.5e-10;
    public static final double kD1 = 0.001;//cpr * 2.337e-7;
    public static final double kFF1 = kv_Vm_r/12.0/ShooterConstants.rpmDenominator;
    public static final double kIz1 = 50.0;
    public static final double kMinOutput1 = -1.0;
    public static final double kMaxOutput1 = 1.0;

    //SPINDOWN
    public static final double kP2 = 0;
    public static final double kI2 = 0;
    public static final double kD2 = 0;
    public static final double kFF2 = kv_Vm_r/12.0/ShooterConstants.rpmDenominator;
    public static final double kIz2 = 50.0;
    public static final double kMinOutput2 = -1.0;
    public static final double kMaxOutput2 = 1.0;

    //PREREV
    public static final double kP3 = 0;
    public static final double kI3 = 0;
    public static final double kD3 = 0;
    public static final double kFF3 = kv_Vm_r/12.0/ShooterConstants.rpmDenominator;
    public static final double kIz3 = 50.0;
    public static final double kMinOutput3 = -1.0;
    public static final double kMaxOutput3 = 1.0;

//Aux wheels
    //SPINUP
    public static final double aP0 = 0; //cpr * 1.827e-7;
    public static final double aI0 = 0;
    public static final double aD0 = 0;
    public static final double aFF0 = ffScale * aux_kv_Vm_r/12.0/ShooterConstants.rpmDenominator;
    public static final double aIz0 = 0;
    public static final double aMinOutput0 = 0.0;
    public static final double aMaxOutput0 = 1.0;
    
    //SHOOTING
    public static final double aP1 = 0.0001;
    public static final double aI1 = 0.0000003;
    public static final double aD1 = 0.0015;//cpr * 2.337e-7;
    public static final double aFF1 = aux_kv_Vm_r/12.0/ShooterConstants.rpmDenominator;
    public static final double aIz1 = 50.0;
    public static final double aMinOutput1 = -1.0;
    public static final double aMaxOutput1 = 1.0;

    //SPINDOWN
    public static final double aP2 = 0;
    public static final double aI2 = 0;
    public static final double aD2 = 0;
    public static final double aFF2 = aux_kv_Vm_r/12.0/ShooterConstants.rpmDenominator;
    public static final double aIz2 = 50.0;
    public static final double aMinOutput2 = -1.0;
    public static final double aMaxOutput2 = 1.0;

    //PREREV
    public static final double aP3 = 0;
    public static final double aI3 = 0;
    public static final double aD3 = 0;
    public static final double aFF3 = aux_kv_Vm_r/12.0/ShooterConstants.rpmDenominator;
    public static final double aIz3 = 50.0;
    public static final double aMinOutput3 = -1.0;
    public static final double aMaxOutput3 = 1.0;
}
