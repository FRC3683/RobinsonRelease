// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.config.ShooterConstants;
import frc.robot.utils.DaveMotorFeedforward;
import frc.robot.utils.MathUtils;
import frc.robot.utils.NPointRemap;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

import com.revrobotics.RelativeEncoder;

public class Shooter extends DaveSubsystem {

    private static Shooter instance;

    public static Shooter GetInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    private Drivetrain drivetrain;
    private Eye eye;

    private final CANSparkMax mrBigSpins;
    private final CANSparkMax msBigSpins;
    private final RelativeEncoder encoder1; // NEO encoder
    private final RelativeEncoder encoder2;
    private final Solenoid hood;

    private SparkMaxPIDController maincontroller;
    private SparkMaxPIDController auxcontroller;

    private NPointRemap shotMap;
    private NPointRemap farshotMap;
    private NPointRemap speedMap;
    private NPointRemap accMap;

    private double targetRPM = 0;
    private double lastRPM = 0;
    private double currentRPM = 0;

    public DoubleLogEntry shotDistLogs;
    public DoubleLogEntry shotRPMLogs;
    public DoubleLogEntry timeLogs;

    public State DISABLED, STOWED, PREREV, REVING, SHOOTING, SPINDOWN,
                 CHAMBER, OPEN_LOOP, REJECTING;

    public double getRPM() {
        return encoder1.getVelocity();
    }

    public double getAuxRPM() {
        return encoder2.getVelocity();
    }

    public void setRPM(double rpm, int pidx) {
        maincontroller.setReference(rpm, ControlType.kVelocity, pidx);
        auxcontroller.setReference(ShooterConstants.auxRPMRatio * rpm, ControlType.kVelocity, pidx);
    }

    public void setDC(double percent) {
        mrBigSpins.set(percent);
        msBigSpins.set(percent);
    }

    public void setTargetRPM(double rpm) {
        this.targetRPM = rpm;
    }

    private Debouncer hoodDebouncer;

    private void hoodActivities(){
        if(eye.getDistance() > ShooterConstants.hoodUpDist + ShooterConstants.hoodHysteresis) hoodUp();
        else if(hoodDebouncer.calculate(eye.getDistance() < ShooterConstants.hoodUpDist - ShooterConstants.hoodHysteresis)) hoodDown();
    }

    public double getTargetRPM() {
        //if(eye.validTarget()){
            double dist = eye.getDistance();
            double res = 0.0;
            if(isHoodUp()) res = farshotMap.calcY(dist);
            else res = shotMap.calcY(dist);
            if(res < 1.0){
                res = ShooterConstants.rpm_rev;
            }
            return res; // + speedMap.calcY(drivetrain.getMps()); 
            //        + 0.0 * speedMap.calcY(drivetrain.getMps()) 
            //        + 0.0 * accMap.calcY(drivetrain.getMps2());
            //}
        //return 0.0;
    }

    public boolean onTarget(){
        return eye.onTarget();
    }

    public boolean onAimlockTarget(){
        return eye.onAimlockTarget();
    }

    public boolean shouldShoot(){
        return MathUtils.closeEnough(getRPM(), targetRPM, 10.0);
    }

    private long prerevCount;

    /** Creates a new Shooter. */
    private Shooter() {
        super("Shooter");
        mrBigSpins = cfg.shooterLefft; // Defines SparkMax for NEOs
        msBigSpins = cfg.shooterMini;

        // Encoders
        encoder1 = mrBigSpins.getEncoder(); // Defines spark encoder
        encoder2 = msBigSpins.getEncoder();

        // Pneumatics
        hood = cfg.hood;

        // Feedback
        maincontroller = mrBigSpins.getPIDController();
        maincontroller.setFeedbackDevice(encoder1);
        maincontroller.setP(ShooterConstants.kP0, 0);
        maincontroller.setI(ShooterConstants.kI0, 0);
        maincontroller.setD(ShooterConstants.kD0, 0);
        maincontroller.setFF(ShooterConstants.kFF0, 0);
        maincontroller.setIZone(ShooterConstants.kIz0, 0);
        maincontroller.setOutputRange(ShooterConstants.kMinOutput0, ShooterConstants.kMaxOutput0, 0);
        maincontroller.setP(ShooterConstants.kP1, 1);
        maincontroller.setI(ShooterConstants.kI1, 1);
        maincontroller.setD(ShooterConstants.kD1, 1);
        maincontroller.setFF(ShooterConstants.kFF1, 1);
        maincontroller.setIZone(ShooterConstants.kIz1, 1);
        maincontroller.setOutputRange(ShooterConstants.kMinOutput1, ShooterConstants.kMaxOutput1, 1);
        maincontroller.setP(ShooterConstants.kP2, 2);
        maincontroller.setI(ShooterConstants.kI2, 2);
        maincontroller.setD(ShooterConstants.kD2, 2);
        maincontroller.setFF(ShooterConstants.kFF2, 2);
        maincontroller.setIZone(ShooterConstants.kIz2, 2);
        maincontroller.setOutputRange(ShooterConstants.kMinOutput2, ShooterConstants.kMaxOutput2, 2);
        maincontroller.setP(ShooterConstants.kP3, 3);
        maincontroller.setI(ShooterConstants.kI3, 3);
        maincontroller.setD(ShooterConstants.kD3, 3);
        maincontroller.setFF(ShooterConstants.kFF3, 3);
        maincontroller.setIZone(ShooterConstants.kIz3, 3);
        maincontroller.setOutputRange(ShooterConstants.kMinOutput3, ShooterConstants.kMaxOutput3, 3);

        auxcontroller = msBigSpins.getPIDController();
        auxcontroller.setFeedbackDevice(encoder2);
        auxcontroller.setP(ShooterConstants.aP0, 0);
        auxcontroller.setI(ShooterConstants.aI0, 0);
        auxcontroller.setD(ShooterConstants.aD0, 0);
        auxcontroller.setFF(ShooterConstants.aFF0, 0);
        auxcontroller.setIZone(ShooterConstants.aIz0, 0);
        auxcontroller.setOutputRange(ShooterConstants.aMinOutput0, ShooterConstants.aMaxOutput0, 0);
        auxcontroller.setP(ShooterConstants.aP1, 1);
        auxcontroller.setI(ShooterConstants.aI1, 1);
        auxcontroller.setD(ShooterConstants.aD1, 1);
        auxcontroller.setFF(ShooterConstants.aFF1, 1);
        auxcontroller.setIZone(ShooterConstants.aIz1, 1);
        auxcontroller.setOutputRange(ShooterConstants.aMinOutput1, ShooterConstants.aMaxOutput1, 1);
        auxcontroller.setP(ShooterConstants.aP2, 2);
        auxcontroller.setI(ShooterConstants.aI2, 2);
        auxcontroller.setD(ShooterConstants.aD2, 2);
        auxcontroller.setFF(ShooterConstants.aFF2, 2);
        auxcontroller.setIZone(ShooterConstants.aIz2, 2);
        auxcontroller.setOutputRange(ShooterConstants.aMinOutput2, ShooterConstants.aMaxOutput2, 2);
        auxcontroller.setP(ShooterConstants.aP3, 3);
        auxcontroller.setI(ShooterConstants.aI3, 3);
        auxcontroller.setD(ShooterConstants.aD3, 3);
        auxcontroller.setFF(ShooterConstants.aFF3, 3);
        auxcontroller.setIZone(ShooterConstants.aIz3, 3);
        auxcontroller.setOutputRange(ShooterConstants.aMinOutput3, ShooterConstants.aMaxOutput3, 3);
        

        targetRPM = 0;
        prerevCount = 0;
        shotMap = new NPointRemap(ShooterConstants.shotmapFile);        
        farshotMap = new NPointRemap(ShooterConstants.farshotmapFile);        
        speedMap = new NPointRemap(ShooterConstants.speedmapFile);
        accMap = new NPointRemap(ShooterConstants.accmapFile);
        hoodDebouncer = new Debouncer(ShooterConstants.hoodDebouncerTime, DebounceType.kRising);

        //Subsystems
        drivetrain = Drivetrain.GetInstance();
        eye = Eye.getInstance();

        // State
        DISABLED = new State("DISABLED", // name displayed to driver station
                () -> {
                    // init
                    eye.look();
                    //eye.close();
                }, () -> {
                    // periodic
                    mrBigSpins.set(0);
                });
        STOWED = new State("STOWED", // name displayed to driver station
                () -> {
                    // init
                    mrBigSpins.set(0);
                    eye.look();
                    //eye.close();
                }, () -> {
                    // periodic
                    mrBigSpins.set(0);
                });
        PREREV = new State("PREREV",
                () -> {
                    // init
                    eye.look();
                    prerevCount = 0;
                }, () -> {
                    // periodic
                    hoodActivities();
                    double alpha = MathUtils.clamp01(++prerevCount / ShooterConstants.prerevSteps);
                    setRPM(alpha * ShooterConstants.rpm_prerev, 3);
                });
        REVING = new State("REVING",
                () -> {
                    // init
                    eye.look();
                }, () -> {
                    // periodic
                    hoodActivities();
                    setRPM(targetRPM, 0);
                });
        SHOOTING = new State("SHOOTING",
                () -> {
                    // init
                    eye.look();
                }, () -> {
                    // periodic
                    hoodActivities();
                    setRPM(targetRPM, 1);
                });
        CHAMBER = new State("CHAMBER",
                () -> {
                    // init
                    eye.look();
                }, () -> {
                    // periodic
                    mrBigSpins.set(0);
                });
        SPINDOWN = new State("SPINDOWN",
                () -> {
                    // init
                    eye.look();
                    //eye.close();
                }, () -> {
                    // periodic
                    if (ShooterConstants.enableSpindown) {
                        setRPM(currentRPM * ShooterConstants.decayRate, 2);
                    } else {
                        setDC(0);
                    }
                });
        OPEN_LOOP = new State("OPEN_LOOP",
                () -> {
                    // init
                    eye.look();
                }, () -> {
                    // periodic

                });
        REJECTING = new State("REJECTING",
                () -> {
                    // init
                    eye.look();
                }, () -> {
                    setRPM(ShooterConstants.rpm_reject, 0);
                });
        setCurrentState(DISABLED);

        // Logging
        AddDashboardEntrySparkEncoderRate("mrBigSpins rate", encoder1);
        //AddDashboardEntrySparkEncoderRate("msBigSpins rate", encoder2);
        AddDashboardEntryWrite("Hood", false, () -> {
            return hood.get();
        });
        AddDashboardEntryWrite("TargetRPM", 0.0, () -> {
            return targetRPM;
        });        
        AddDashboardEntryWrite("tx", 0.0, () -> {
            return eye.getHeadingCorrection();
        });        
        AddDashboardEntryWrite("tv", 0.0, () -> {
            return eye.validTarget();
        });                
        AddDashboardEntryWrite("onTarget", false, () -> {
            return onTarget();
        });
        AddDashboardEntryWrite("dist", 0.0, () -> {
            return eye.getDistance();
        });
        AddDashboardEntryWrite("getTargetRPM", 0.0, () -> {
            return getTargetRPM();
        });
        AddDashboardEntryState(DISABLED);

        //logging
        DataLog log = DataLogManager.getLog();
        shotDistLogs = new DoubleLogEntry(log, "shot distances");
        shotRPMLogs = new DoubleLogEntry(log, "shot rpm target");
        timeLogs = new DoubleLogEntry(log, "shot time");
    }

    public void toggleHood(){
        hood.set(!hood.get());
    }

    public boolean isHoodUp(){
        return hood.get();
    }

    public void hoodUp(){
        hood.set(true);
    }

    public void hoodDown(){
        hood.set(false);
    }

    public boolean atTargetRPM(){
        return MathUtils.closeEnoughPercent(targetRPM, currentRPM, 0.1);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        currentRPM = getRPM();
        super.periodic();
        lastRPM = currentRPM;
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
