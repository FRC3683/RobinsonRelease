package frc.robot.config;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DrivetrainConstants {
    //Specs
    public static final double maxVolts = 12.0;
    public static final double timeToFullOpen = 0.0;
    public static final double timeToFullClosed = 0.0;
    public static final int currentLimit = 55;
    public static final boolean gyroReversed = true;

    //Physics
    public static final double dt_s = 0.02; // s
    public static final double wheelDiameter_m = 0.1 * 105/120; // meters
    public static final double trackwidth_m = Units.inchesToMeters(24.340); // meters
    public static final DCMotor gearbox = DCMotor.getNEO(2);    
    public static final double gearingH = 5.04;   // Reduction
    public static final double gearingL = 12.35;    // Reduction
    public static final double MoI_Kgm2 = 7.74;  // Kg *m^2
    public static final double mass_Kg = Units.lbsToKilograms(145); // Mass of entire robot, Kg

    //Feedforward
    public static final double ks_V = 0.18287;         //fake, get from characterization
    public static final double kv_Vs_m = 5.5416;      //fake, get from characterization
    public static final double ka_Vs2_m = 0.57926;      //fake, get from characterization
    public static final double kv_Vs_deg = 0.13342;     //fake, get from characterization
    public static final double ka_Vs2_deg = 0.00017229;    //fake, get from characterization

    //Feedback
    public static final double kPVelocity = 7.2658;
    public static final double kIVelocity = 0;
    public static final double kDVelocity = 0;
    public static final double kPSpark = 0;
    public static final double kISpark = 0;
    public static final double kDSpark = 0;
    public static final double kFFSpark = 0;
    public static final double kIzSpark = 0;
    public static final double kMinOutputSpark = -1.0;
    public static final double kMaxOutputSpark = 1.0;
    public static final double kP_STTBOLLS = 0.055; 
    public static final double kI_STTBOLLS = 0.0; //0.11;
    public static final double kD_STTBOLLS = 0.0;//4.4;  
    public static final double kIRange_STTBOLLS = 1.0;
    public static final double min_out_STTBOLLS = 0.18287; 
    public static final double kP_aSTTBOLLS = 0.05; 
    public static final double kI_aSTTBOLLS = 0.0; //0.11;
    public static final double kD_aSTTBOLLS = 0.0;//4.4;  
    public static final double kIRange_aSTTBOLLS = 1.0;
    public static final double min_out_aSTTBOLLS = 0.18287;       
    public static final double kP_Aimlock = 0.12; //0.06;
    public static final double kI_Aimlock = 0.0; //0.11;
    public static final double kD_Aimlock = 0.0; 
    public static final double kIRange_Aimlock = 1.0;
    public static final double kP_Baselock_mps = 0.0;
    public static final double kI_Baselock_mps = 0.00;
    public static final double kD_Baselock_mps = 0.00;
    public static final double kIRange_Baselock_mps = 1.0;
    public static final double kP_Baselock_deg = 0.0;
    public static final double kI_Baselock_deg = 0.00;
    public static final double kD_Baselock_deg = 0.00;
    public static final double kIRange_Baselock_deg = 1.0;
    public static final double aimlock_rate = 1.0;    
    public static final double baselock_rate = 0.3;    
    public static final double kP_Turn = 0.00;
    public static final double kI_Turn = 0.00;
    public static final double kD_Turn = 0.00;
    public static final double vel_Turn = 10.00;
    public static final double acc_Turn = 5.00;

    //Auto
    public static final double autoMaxVoltage_V = 11; // volts
    public static final double autoMaxVelocity_m_s = 3; // m/s
    public static final double autoMaxAcceleration_m_s2 = 1; // m/s2
    public static final double kB = 2;
    public static final double kZeta = 0.7;
}
