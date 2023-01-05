package frc.robot.config;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SerialPort.Port;

public class Config {
    private static Config instance;

    public static Config getInstance() {
        if (instance == null) {
            instance = new Config();
        }
        return instance;
    }

    public static boolean performanceMode = false;
    private static boolean burnFlash = true;

    // Ports
    public static final int driveLefft1ID = 2;
    public static final int driveLefft2ID = 3;
    public static final int driveRight1ID = 4;
    public static final int driveRight2ID = 5;
    public static final int gearShiftID = 1;

    public static final int shooterLefftID = 6;
    public static final int shooterRightID = 7;
    public static final int shooterMiniID = 8;
    public static final int hoodID = 3;

    public static final int intakeLefftID = 12;
    public static final int intakeRightID = 13;
    public static final int conveyorID = 11;
    public static final int intakePistonID = 0;
    public static final int gatePistonID = 2;

    public static final int beamID = 0;

    public static final int leftSpoolID = 9;
    public static final int rightSpoolID = 10;
    public static final int latchFwdID = 7;
    public static final int latchRevID = 6;
    public static final int armFwdID = 5;
    public static final int armRevID = 4;

    public static final int gateBeamID = 1;
    public static final int intakeBeamID = 0;

    private Config() {
        // Calibration delay
        if(burnFlash){
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        /**
         * Drivetrain
         */
        driveLefft1 = new CANSparkMax(driveLefft1ID, MotorType.kBrushless);
        driveLefft2 = new CANSparkMax(driveLefft2ID, MotorType.kBrushless);
        driveRight1 = new CANSparkMax(driveRight1ID, MotorType.kBrushless);
        driveRight2 = new CANSparkMax(driveRight2ID, MotorType.kBrushless);

        driveLefft1.clearFaults();
        driveLefft2.clearFaults();
        driveRight1.clearFaults();
        driveRight2.clearFaults();
        
        driveLefft1.restoreFactoryDefaults(false);
        driveLefft2.restoreFactoryDefaults(false);
        driveRight1.restoreFactoryDefaults(false);
        driveRight2.restoreFactoryDefaults(false);

        driveLefft1.enableVoltageCompensation(DrivetrainConstants.maxVolts);
        driveLefft2.enableVoltageCompensation(DrivetrainConstants.maxVolts);
        driveRight1.enableVoltageCompensation(DrivetrainConstants.maxVolts);
        driveRight2.enableVoltageCompensation(DrivetrainConstants.maxVolts);

        driveLefft1.setOpenLoopRampRate(DrivetrainConstants.timeToFullOpen);
        driveLefft2.setOpenLoopRampRate(DrivetrainConstants.timeToFullOpen);
        driveRight1.setOpenLoopRampRate(DrivetrainConstants.timeToFullOpen);
        driveRight2.setOpenLoopRampRate(DrivetrainConstants.timeToFullOpen);
        driveLefft1.setClosedLoopRampRate(DrivetrainConstants.timeToFullClosed);
        driveLefft2.setClosedLoopRampRate(DrivetrainConstants.timeToFullClosed);
        driveRight1.setClosedLoopRampRate(DrivetrainConstants.timeToFullClosed);
        driveRight2.setClosedLoopRampRate(DrivetrainConstants.timeToFullClosed);

        driveLefft1.setSmartCurrentLimit(DrivetrainConstants.currentLimit);
        driveLefft2.setSmartCurrentLimit(DrivetrainConstants.currentLimit);
        driveRight1.setSmartCurrentLimit(DrivetrainConstants.currentLimit);
        driveRight2.setSmartCurrentLimit(DrivetrainConstants.currentLimit);

        driveLefft1.setIdleMode(IdleMode.kCoast); // Set brake or coast
        driveLefft2.setIdleMode(IdleMode.kCoast); // Set brake or coast
        driveRight1.setIdleMode(IdleMode.kCoast); // Set brake or coast
        driveRight2.setIdleMode(IdleMode.kCoast); // Set brake or coast
        
        driveLefft2.follow(driveLefft1, false);
        driveRight2.follow(driveRight1, false);
        driveLefft1.setInverted(true);
        driveRight1.setInverted(false);

        driveLefft1.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
        driveLefft1.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        driveLefft1.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
        driveLefft2.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
        driveLefft2.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 1000);
        driveLefft2.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 1000);
        driveRight1.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
        driveRight1.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        driveRight1.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
        driveRight2.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
        driveRight2.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 1000);
        driveRight2.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 1000);

        navx = new AHRS(Port.kUSB);
        gearShift = new Solenoid(PneumaticsModuleType.CTREPCM, gearShiftID);

        /**
         * Shooter
         */
        shooterLefft = new CANSparkMax(shooterLefftID, MotorType.kBrushless);
        shooterRight = new CANSparkMax(shooterRightID, MotorType.kBrushless);
        shooterMini = new CANSparkMax(shooterMiniID, MotorType.kBrushless);

        shooterLefft.clearFaults();
        shooterRight.clearFaults();
        shooterMini.clearFaults();

        shooterLefft.restoreFactoryDefaults(false);
        shooterRight.restoreFactoryDefaults(false);
        shooterMini.restoreFactoryDefaults(false);

        shooterLefft.enableVoltageCompensation(ShooterConstants.maxVolts);
        shooterRight.enableVoltageCompensation(ShooterConstants.maxVolts);
        shooterMini.enableVoltageCompensation(ShooterConstants.maxVolts);

        shooterLefft.setSmartCurrentLimit(ShooterConstants.currentLimit);
        shooterRight.setSmartCurrentLimit(ShooterConstants.currentLimit);
        shooterMini.setSmartCurrentLimit(ShooterConstants.currentLimit);

        shooterLefft.setIdleMode(IdleMode.kCoast); // Set brake or coast
        shooterRight.setIdleMode(IdleMode.kCoast); // Set brake or coast
        shooterMini.setIdleMode(IdleMode.kCoast); // Set brake or coast

        shooterRight.follow(shooterLefft, true);
        shooterLefft.setInverted(false);
        shooterMini.setInverted(false);

        shooterLefft.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);
        shooterLefft.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        shooterLefft.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 1000);        
        shooterRight.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
        shooterRight.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 1000);
        shooterRight.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 1000);
        shooterMini.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
        shooterMini.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        shooterMini.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 1000); 

        hood = new Solenoid(PneumaticsModuleType.CTREPCM, hoodID);

        /**
         * Inveyor
         */
        intakeLefft = new VictorSPX(intakeLefftID);
        intakeRight = new VictorSPX(intakeRightID);
        
        intakeLefft.configFactoryDefault();
        intakeRight.configFactoryDefault();
        
        intakeLefft.setNeutralMode(NeutralMode.Brake);
        intakeRight.setNeutralMode(NeutralMode.Brake);
        
        intakeLefft.configOpenloopRamp(InveyorConstants.timeToFullOpen);
        intakeRight.configOpenloopRamp(InveyorConstants.timeToFullOpen);
        
        intakeLefft.configVoltageCompSaturation(InveyorConstants.maxVolts);
        intakeRight.configVoltageCompSaturation(InveyorConstants.maxVolts);
        
        intakeLefft.enableVoltageCompensation(true);
        intakeRight.enableVoltageCompensation(true);
        
        intakeRight.setInverted(true);
        intakeLefft.follow(intakeRight);
        intakeLefft.setInverted(InvertType.OpposeMaster);
        
        conveyor = new TalonSRX(conveyorID);
        conveyor.configFactoryDefault();
        conveyor.setNeutralMode(NeutralMode.Brake);
        conveyor.configOpenloopRamp(InveyorConstants.timeToFullOpen);
        conveyor.configVoltageCompSaturation(InveyorConstants.maxVolts);
        conveyor.enableVoltageCompensation(true);
        conveyor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        conveyor.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_25Ms);
        conveyor.configVelocityMeasurementWindow(16);
        conveyor.setSensorPhase(false);
        conveyor.setInverted(false);
        conveyor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10);
        conveyor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10);
        conveyor.configNominalOutputForward(0);
        conveyor.configNominalOutputReverse(0);
        conveyor.configPeakOutputForward(1);
        conveyor.configPeakOutputReverse(-1);

        intakePiston = new Solenoid(PneumaticsModuleType.CTREPCM, intakePistonID);
        gatePiston = new Solenoid(PneumaticsModuleType.CTREPCM, gatePistonID);
        // beam = new DigitalInput(beamID);
        //colour = new ColorSensorV3(I2C.Port.kOnboard);
        gateBeam = new DigitalInput(gateBeamID);
        intakeBeam = new DigitalInput(intakeBeamID);

        /**
         * Climb
         */
        leftSpool = new TalonSRX(leftSpoolID);
        rightSpool = new VictorSPX(rightSpoolID);

        leftSpool.configFactoryDefault();
        rightSpool.configFactoryDefault();

        
        leftSpool.configVoltageCompSaturation(ClimbConstants.maxVolts);
        leftSpool.enableVoltageCompensation(true);        
        rightSpool.configVoltageCompSaturation(ClimbConstants.maxVolts);
        rightSpool.enableVoltageCompensation(true);

        rightSpool.follow(leftSpool);
        rightSpool.setInverted(InvertType.OpposeMaster);
        leftSpool.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        leftSpool.setSensorPhase(true);
        leftSpool.setInverted(false);
        leftSpool.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10);
        leftSpool.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10);
        leftSpool.configNominalOutputForward(0);
        leftSpool.configNominalOutputReverse(0);
        leftSpool.configPeakOutputForward(1);
        leftSpool.configPeakOutputReverse(-1);
        latch = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, latchFwdID, latchRevID);
        arm = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, armFwdID, armRevID);

        // Calibration delay
        if(burnFlash){
            try {
                driveLefft1.burnFlash();
                driveLefft2.burnFlash();
                driveRight1.burnFlash();
                driveRight2.burnFlash();
                
                shooterLefft.burnFlash();
                shooterRight.burnFlash();
                shooterMini.burnFlash();
                
                
                Thread.sleep(5000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
            
    }

    // Drivetrain
    public final CANSparkMax driveLefft1;
    public final CANSparkMax driveLefft2;
    public final CANSparkMax driveRight1;
    public final CANSparkMax driveRight2;
    public final AHRS navx;
    public final Solenoid gearShift;

    // Shooter
    public final CANSparkMax shooterLefft;
    public final CANSparkMax shooterRight;
    public final CANSparkMax shooterMini;
    public final Solenoid hood;

    // Inveyor
    public final VictorSPX intakeLefft;
    public final VictorSPX intakeRight;
    public final TalonSRX conveyor;
    public final Solenoid intakePiston;
    public final Solenoid gatePiston;
    public final DigitalInput gateBeam;
    public final DigitalInput intakeBeam;

    // Climb
    public final TalonSRX leftSpool;
    public final VictorSPX rightSpool;
    public final DoubleSolenoid latch;
    public final DoubleSolenoid arm;
}
