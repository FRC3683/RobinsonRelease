// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.commands.SparkRamseteCommand;
import frc.robot.config.DrivetrainConstants;
import frc.robot.utils.DaveMotorFeedforward;
import frc.robot.utils.MathUtils;
import frc.robot.utils.OI;

public class Drivetrain extends DaveSubsystem {

    private static Drivetrain instance;

    public static Drivetrain GetInstance() {
        if (instance == null) {
            instance = new Drivetrain();
        }
        return instance;
    }

    public final CANSparkMax mLefft1;
    public final CANSparkMax mLefft2;
    public final CANSparkMax mRight1;
    public final CANSparkMax mRight2;

    public final RelativeEncoder eLefft;
    public final RelativeEncoder eRight;

    public final AHRS navx;

    public final Solenoid gearShift;

    public DifferentialDrive diffyDrive;
    public DifferentialDriveOdometry diffyOdom;
    public DifferentialDriveKinematics diffyKin;
    public DaveMotorFeedforward ff;
    public PIDController lefftCon;
    public PIDController rightCon;
    public SparkMaxPIDController lefftSparkCon;
    public SparkMaxPIDController rightSparkCon;
    public RamseteController gordon;
    public PIDController sttbollsCon;
    public PIDController asttbollsCon;
    public PIDController aimlockCon;
    public PIDController baselockMpsCon;
    public PIDController baselockDegCon;
    public ProfiledPIDController turnCon;
    public DaveMotorFeedforward turnff;

    private final Eye eye;
    private final OI oi;

    private Field2d field;

    private SlewRateLimiter sttbolls_limiter;
    private SlewRateLimiter baselock_limiter;

    private double fwd, turn, targetAngle;

    private Pose2d pose;

    public Pose2d getPose() {
        return pose;
    }

    public void writeMpsToSparks(double lefftVel_mps, double rightVel_mps, double prev_lefftVel_mps,
            double prev_rightVel_mps) {
        double lv = lefftVel_mps, la = (lefftVel_mps - prev_lefftVel_mps) / DrivetrainConstants.dt_s;
        double lArbFF_V = ff.stiction(lv, la) + ff.accff(lv, la);
        double lefftVel_rpm = metersToRevs(lv) * 60 * currentGearing();
        lefftSparkCon.setReference(lefftVel_rpm, ControlType.kVelocity, 0, lArbFF_V, ArbFFUnits.kVoltage);

        double rv = rightVel_mps, ra = (rightVel_mps - prev_rightVel_mps) / DrivetrainConstants.dt_s;
        double rArbFF_V = ff.stiction(rv, ra) + ff.accff(rv, ra);
        double rightVel_rpm = metersToRevs(rv) * 60 * currentGearing();
        rightSparkCon.setReference(rightVel_rpm, ControlType.kVelocity, 0, rArbFF_V, ArbFFUnits.kVoltage);

        diffyDrive.feed();
    }

    public void writeVolts(double lefftVolts, double rightVolts) {
        double batteryVoltage = RobotController.getBatteryVoltage();
        if (Math.max(Math.abs(lefftVolts), Math.abs(rightVolts)) > batteryVoltage) {
            lefftVolts *= batteryVoltage / 12.0;
            rightVolts *= batteryVoltage / 12.0;
        }
        mLefft1.set(-lefftVolts / 12.0);
        mRight1.set(-rightVolts / 12.0);
        diffyDrive.feed();
    }

    public void setMaxOutput(double output) {
        diffyDrive.setMaxOutput(output);
    }

    public void zeroHeading() {
        navx.reset();
    }

    // degrees from -180 to 180
    public double getHeadingDeg() {
        // return 0;
        return Math.IEEEremainder(navx.getAngle(), 360) *
                (DrivetrainConstants.gyroReversed ? -1.0 : 1.0);
    }

    private double last_degps;
    private double degps;

    public double getAngularRate() {
        // return 0;
        return navx.getRate() * (DrivetrainConstants.gyroReversed ? -1.0 : 1.0);
    }

    public double getAngularAcc() {
        return (degps - last_degps) / DrivetrainConstants.dt_s;
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeadingDeg());
    }

    public double getVoltageAvgLefft() {
        return mLefft1.getBusVoltage();
    }

    public double getVoltageAvgRight() {
        return mRight1.getBusVoltage();
    }

    private void lGear() {
        gearShift.set(false); // TODO check actual values
    }

    private void hGear() {
        gearShift.set(true); // TODO check actual values
    }

    // convention: piston engaged = low gear
    public double currentGearing() {
        return gearShift.get() ? DrivetrainConstants.gearingH : DrivetrainConstants.gearingL;
    }

    private void coastMode() {
        // allows for smoother, less accurate movement
        mLefft1.setIdleMode(IdleMode.kCoast);
        mLefft2.setIdleMode(IdleMode.kCoast);
        mRight1.setIdleMode(IdleMode.kCoast);
        mRight2.setIdleMode(IdleMode.kCoast);
    }

    private void brakeMode() {
        // allows for less smooth, more acurate movement
        mLefft1.setIdleMode(IdleMode.kBrake);
        mLefft2.setIdleMode(IdleMode.kBrake);
        mRight1.setIdleMode(IdleMode.kBrake);
        mRight2.setIdleMode(IdleMode.kBrake);
    }

    public void splitStickDrive(double fwd, double turn) {
        this.fwd = fwd;
        this.turn = turn;
    }

    public void chesseyDrive() {
        diffyDrive.tankDrive(MathUtils.calcLeftDrive(fwd, turn), MathUtils.calcRightDrive(fwd, turn));
    }

    // -1 to 1, - left, + right
    public void turnAngle(double angle) {
        diffyDrive.arcadeDrive(0, angle);
    }

    public void resetEncoders() {
        eLefft.setPosition(0);
        eRight.setPosition(0);
    }

    public void setRawLefft(double pos) {
        eLefft.setPosition(-pos);
    }

    public void setRawRight(double pos) {
        eRight.setPosition(-pos);
    }

    public void setMetersLefft(double pos) {
        setRawLefft(metersToRevs(pos));
    }

    public void setMetersRight(double pos) {
        setRawRight(metersToRevs(pos));
    }

    public double getRawLefft() {
        return -eLefft.getPosition();
    }

    public double getRawRight() {
        return -eRight.getPosition();
    }

    public double getRawRateLefft() {
        return -eLefft.getVelocity();
    }

    public double getRawRateRight() {
        return -eRight.getVelocity();
    }

    public double getRevsLefft() {
        return getRawLefft() / currentGearing();
    }

    public double getRevsRight() {
        return getRawRight() / currentGearing();
    }

    public double getRpmLefft() {
        return getRawRateLefft() / currentGearing();
    }

    public double getRpmRight() {
        return getRawRateRight() / currentGearing();
    }

    private double revsToMeters(double revs) {
        return (DrivetrainConstants.wheelDiameter_m * Math.PI) / currentGearing() * revs;
    }

    private double metersToRevs(double meters) {
        return currentGearing() / (DrivetrainConstants.wheelDiameter_m * Math.PI) * meters;
    }

    public double getMetersLefft() {
        return revsToMeters(getRawLefft());
    }

    public double getMetersRight() {
        return revsToMeters(getRawRight());
    }

    public double getMpsLefft() {
        return revsToMeters(getRawRateLefft()) / 60.0;
    }

    public double getMpsRight() {
        return revsToMeters(getRawRateRight()) / 60.0;
    }

    private double last_mps;
    private double mps;

    public double getMps() {
        return revsToMeters(0.5 * (getMpsLefft() + getMpsRight()));
    }

    public double getMps2() {
        return (mps - last_mps) / DrivetrainConstants.dt_s;
    }

    /**
     * Returns the current wheel speeds of the robot in meters/second.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getMpsLefft(), getMpsRight());
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        this.pose = pose;
        resetEncoders();
        navx.setAngleAdjustment(-pose.getRotation().getDegrees());
        diffyOdom.resetPosition(pose, pose.getRotation());
    }

    public void resetNavx(double degrees){
        navx.setAngleAdjustment(-degrees);
    }

    public void setRamsete(boolean set){
        gordon.setEnabled(set);
    }

    // Generates Ramsete command for auto
    public RamseteCommand generateRamseteCommand(Trajectory traj) {
        return new RamseteCommand(
                traj,
                this::getPose,
                gordon,
                ff,
                diffyKin,
                this::getWheelSpeeds,
                lefftCon,
                rightCon,
                this::writeVolts,
                this);
    }

    public void disableTelemetry(){        
        mLefft1.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 2000);
        mRight1.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 2000);
    }

    // Generates Ramsete command for auto
    public SparkRamseteCommand generateSparkRamseteCommand(Trajectory traj) {
        return new SparkRamseteCommand(
                traj,
                this::getPose,
                gordon,
                diffyKin,
                this::writeMpsToSparks,
                this);
    }

    public State HGOL;
    public State LGOL;
    public State CLIMB;
    public State HGCL;
    public State LGCL;
    public State TURNCL;
    public State STTBOLLS;
    public State ASTTBOLLS;
    public State AIMLOCK;
    public State BASELOCK;
    public State DISABLED;

    /** Creates a new Drivetrain. */
    private Drivetrain() {
        super("Drivetrain");

        // Motors
        mLefft1 = cfg.driveLefft1;
        mLefft2 = cfg.driveLefft2;
        mRight1 = cfg.driveRight1;
        mRight2 = cfg.driveRight2;

        // Encoders
        eLefft = mLefft1.getEncoder();
        eRight = mRight1.getEncoder();

        // Navx
        navx = cfg.navx;

        // Pneumatics
        gearShift = cfg.gearShift;

        // Odometry/Kinematics
        diffyDrive = new DifferentialDrive(mLefft1, mRight1);
        diffyKin = new DifferentialDriveKinematics(DrivetrainConstants.trackwidth_m);
        diffyOdom = new DifferentialDriveOdometry(getRotation2d());

        // Controllers
        ff = new DaveMotorFeedforward(DrivetrainConstants.ks_V,
                DrivetrainConstants.kv_Vs_m,
                DrivetrainConstants.ka_Vs2_m);
        lefftCon = new PIDController(DrivetrainConstants.kPVelocity,
                DrivetrainConstants.kIVelocity,
                DrivetrainConstants.kDVelocity);
        rightCon = new PIDController(DrivetrainConstants.kPVelocity,
                DrivetrainConstants.kIVelocity,
                DrivetrainConstants.kDVelocity);
        lefftSparkCon = mLefft1.getPIDController();
        lefftSparkCon.setP(DrivetrainConstants.kPSpark);
        lefftSparkCon.setI(DrivetrainConstants.kISpark);
        lefftSparkCon.setD(DrivetrainConstants.kDSpark);
        lefftSparkCon.setIZone(DrivetrainConstants.kIzSpark);
        lefftSparkCon.setFF(DrivetrainConstants.kFFSpark);
        lefftSparkCon.setOutputRange(DrivetrainConstants.kMinOutputSpark,
                DrivetrainConstants.kMaxOutputSpark);
        lefftSparkCon.setFeedbackDevice(eLefft);
        rightSparkCon = mRight1.getPIDController();
        rightSparkCon.setP(DrivetrainConstants.kPSpark);
        rightSparkCon.setI(DrivetrainConstants.kISpark);
        rightSparkCon.setD(DrivetrainConstants.kDSpark);
        rightSparkCon.setIZone(DrivetrainConstants.kIzSpark);
        rightSparkCon.setFF(DrivetrainConstants.kFFSpark);
        rightSparkCon.setOutputRange(DrivetrainConstants.kMinOutputSpark,
                DrivetrainConstants.kMaxOutputSpark);
        rightSparkCon.setFeedbackDevice(eRight);
        gordon = new RamseteController(DrivetrainConstants.kB, DrivetrainConstants.kZeta);
        gordon.setEnabled(true); // TODO: remove
        sttbollsCon = new PIDController(DrivetrainConstants.kP_STTBOLLS, DrivetrainConstants.kI_STTBOLLS,
                DrivetrainConstants.kD_STTBOLLS);
        sttbollsCon.setIntegratorRange(-DrivetrainConstants.kIRange_STTBOLLS, DrivetrainConstants.kIRange_STTBOLLS);
        asttbollsCon = new PIDController(DrivetrainConstants.kP_aSTTBOLLS, DrivetrainConstants.kI_aSTTBOLLS,
                DrivetrainConstants.kD_aSTTBOLLS);
        asttbollsCon.setIntegratorRange(-DrivetrainConstants.kIRange_aSTTBOLLS, DrivetrainConstants.kIRange_aSTTBOLLS);
        aimlockCon = new PIDController(DrivetrainConstants.kP_Aimlock, DrivetrainConstants.kI_Aimlock,
                DrivetrainConstants.kD_Aimlock);
        aimlockCon.setIntegratorRange(-DrivetrainConstants.kIRange_Aimlock, DrivetrainConstants.kIRange_Aimlock);
        baselockMpsCon = new PIDController(DrivetrainConstants.kP_Baselock_mps, DrivetrainConstants.kI_Baselock_mps,
                DrivetrainConstants.kD_Baselock_mps);
        baselockMpsCon.setIntegratorRange(-DrivetrainConstants.kIRange_Baselock_mps, DrivetrainConstants.kIRange_Baselock_mps);
        baselockDegCon = new PIDController(DrivetrainConstants.kP_Baselock_deg, DrivetrainConstants.kI_Baselock_deg,
                DrivetrainConstants.kD_Baselock_deg);
        baselockDegCon.setIntegratorRange(-DrivetrainConstants.kIRange_Baselock_deg, DrivetrainConstants.kIRange_Baselock_deg);
        
        turnCon = new ProfiledPIDController(DrivetrainConstants.kP_Turn,  
                                            DrivetrainConstants.kI_Turn, 
                                            DrivetrainConstants.kD_Turn, 
                new TrapezoidProfile.Constraints(DrivetrainConstants.vel_Turn, 
                                                 DrivetrainConstants.acc_Turn));
        
        turnCon.enableContinuousInput(-180.0, 180.0);
        
        turnff = new DaveMotorFeedforward(DrivetrainConstants.ks_V,
                DrivetrainConstants.kv_Vs_deg,
                DrivetrainConstants.ka_Vs2_deg);


        sttbolls_limiter = new SlewRateLimiter(25);
        baselock_limiter = new SlewRateLimiter(25);

        eye = Eye.getInstance();
        oi = OI.getInstance();
        field = new Field2d();

        // Zero
        resetEncoders();
        pose = new Pose2d();
        diffyOdom.resetPosition(pose, getRotation2d());
        mps = 0.0;
        last_mps = 0.0;
        degps = 0.0;
        last_degps = 0.0;
        fwd = 0.0;
        turn = 0.0;
        targetAngle = 0.0;

        // State
        HGOL = new State("HIGH GEAR OPEN LOOP",
                () -> {
                    // init
                    coastMode();
                    hGear();
                    oi.rumbleDriver(0.0, 0.0);
                    eye.look();
                }, () -> {
                    // periodic
                    chesseyDrive();
                });
        LGOL = new State("LOW GEAR OPEN LOOP",
                () -> {
                    // init
                    coastMode();
                    lGear();
                    oi.rumbleDriver(0.0, 0.0);
                    eye.look();
                }, () -> {
                    // periodic
                    turn *= 0.65;
                    chesseyDrive();
                });
        CLIMB = new State("CLIMB",
                () -> {
                    // init
                    coastMode();
                    lGear();
                    oi.rumbleDriver(0.0, 0.0);
                    eye.close();
                }, () -> {
                    // periodic
                    fwd *= 0.65;
                    turn *= 0.65;
                    chesseyDrive();
                });
        HGCL = new State("HIGH GEAR CLOSED LOOP",
                () -> {
                    // init
                    hGear();
                    brakeMode();
                    oi.rumbleDriver(0.0, 0.0);
                }, () -> {
                    // periodic
                    diffyDrive.feed();
                });
        LGCL = new State("LOW GEAR CLOSED LOOP",
                () -> {
                    // init
                    lGear();
                    brakeMode();
                    oi.rumbleDriver(0.0, 0.0);
                }, () -> {
                    // periodic
                    diffyDrive.feed();


                });
        TURNCL = new State("TURNCL",
                () -> {
                    // init
                    lGear();
                    brakeMode();
                    turnCon.setGoal(targetAngle);
                }, () -> {
                    // periodic
                    double a = getAngularAcc();




                    turn = MathUtils.clamp1(turnCon.calculate(getHeadingDeg(), targetAngle) 
                         + turnff.calculate(turnCon.getSetpoint().velocity, a));
                    fwd = 0.0;
                    if (turn > 0.0)
                        oi.rumbleDriver(MathUtils.clamp01(0.0), MathUtils.clamp01(turn));
                    else
                        oi.rumbleDriver(MathUtils.clamp01(-turn), MathUtils.clamp01(0.0));

                    chesseyDrive();
                });
        STTBOLLS = new State("STTBOLLS", // some turning thing based on limelight stuff
                () -> {
                    // init
                    lGear();
                    brakeMode();
                    sttbollsCon.reset();
                }, () -> {
                    // periodic

                    if (eye.validTarget()) {
                        turn = MathUtils.clamp1(-sttbollsCon.calculate(eye.getHeadingCorrection(), 0.0));

                        oi.rumbleDriver(0.5, 0.5);
                        fwd = 0.0;
                    } else {
                        oi.rumbleDriver(0.0, 0.0);                        
                        sttbollsCon.reset();
                        turn *= 0.65;
                    }
                    chesseyDrive();
                });
        ASTTBOLLS = new State("ASTTBOLLS", // some turning thing based on limelight stuff
                () -> {
                    // init
                    lGear();
                    brakeMode();
                    asttbollsCon.reset();
                }, () -> {
                    // periodic

                    if (eye.validTarget()) {
                        turn = MathUtils.clamp1(-asttbollsCon.calculate(eye.getHeadingCorrection(), 0.0));

                        oi.rumbleDriver(0.5, 0.5);
                        fwd = 0.0;
                    } else {
                        oi.rumbleDriver(0.0, 0.0);                        
                        asttbollsCon.reset();
                    }
                    chesseyDrive();
                });
        AIMLOCK = new State("AIMLOCK", // some turning thing based on limelight stuff
                () -> {
                    // init
                    lGear();
                    brakeMode();
                    aimlockCon.reset();
                }, () -> {
                    // periodic

                    if (eye.validTarget()) {
                        turn = MathUtils.clamp1(-aimlockCon.calculate(eye.getHeadingCorrection(), 0.0));

                        oi.rumbleDriver(0.75, 0.75);
                        fwd = 0.0;
                    } else {
                        oi.rumbleDriver(0.0, 0.0);                        
                        sttbollsCon.reset();                        
                        turn *= 0.65;
                    }
                    chesseyDrive();
                });

        BASELOCK = new State("BASELOCK",
                () -> {
                    // init
                    lGear();
                    brakeMode();
                    baselockMpsCon.reset();
                    baselockDegCon.reset();
                    baselockDegCon.setSetpoint(getHeadingDeg());
                }, () -> {
                    // periodic

                    if (eye.validTarget()) {
                        turn = MathUtils.clamp1(-baselockDegCon.calculate(getHeadingDeg()));
                        fwd = MathUtils.clamp1(-baselockMpsCon.calculate(getMps(), 0.0));
                        oi.rumbleDriver(1.0, 1.0);
                    } else {
                        oi.rumbleDriver(0.0, 0.0);
                        turn *= 0.65;
                    }
                    chesseyDrive();
                });

        DISABLED = new State("DISABLED",
                () -> {
                    // init
                    hGear();
                    coastMode();

                    fwd = 0.0;
                    turn = 0.0;
                }, () -> {
                    // periodic
                    diffyDrive.tankDrive(0.0, 0.0);
                });
        setCurrentState(DISABLED);

        // Logging
        AddDashboardEntryState(DISABLED);

        AddDashboardEntryWrite("left m pos", 0.0, () -> {
            return getMetersLefft();
        });
        AddDashboardEntryWrite("right m pos", 0.0, () -> {
            return getMetersRight();
        });
        AddDashboardEntryWrite("navx Yaw", 0.0, () -> {
            return getHeadingDeg();
        });
        AddDashboardEntryWrite("navx angular rate", 0.0, () -> {
            return getAngularRate();
        });
        AddDashboardEntryWrite("navx Pitch", 0.0, () -> {
            return navx.getPitch();
        });

        AddDashboardEntryWrite("fwd", 0.0, () -> {
            return fwd;
        });

        AddDashboardEntryWrite("turn", 0.0, () -> {
            return turn;
        });

        AddDashboardEntryWrite("shouldAimlock", false, () -> {
            return shouldAimlock();
        });

        AddDashboardEntryWrite("shouldBaselock", false, () -> {
            return shouldBaselock();
        });

        AddDashboardEntryWrite("Speed", 0.0, () -> {
            return getMps();
        });
        AddDashboardEntryWrite("Acc", 0.0, () -> {
            return getMps2();
        });

        SmartDashboard.putData("Field", field);
    }

    public void setTargetAngle(double ang) {
        targetAngle = Math.IEEEremainder(ang, 360);
    }

    public boolean shouldAimlock(){
        return eye.validTarget() && Math.abs(getAngularRate()) < DrivetrainConstants.aimlock_rate && eye.onAimlockTarget();
    }

    public boolean shouldBaselock(){
        return eye.validTarget() && Math.abs(getAngularRate()) < DrivetrainConstants.baselock_rate && eye.onBaselockTarget();
    }

    public boolean validTarget(){
        return eye.validTarget();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        last_mps = mps;
        last_degps = degps;
        mps = getMps();
        degps = turnCon.getSetpoint().velocity;
        super.periodic();
        pose = diffyOdom.update(getRotation2d(), getMetersLefft(), getMetersRight());
        field.setRobotPose(pose);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        super.simulationPeriodic();
    }

}
