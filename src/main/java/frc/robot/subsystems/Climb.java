package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.config.ClimbConstants;

public class Climb extends DaveSubsystem {

    private static Climb instance;

    public static Climb GetInstance() {
        if (instance == null) {
            instance = new Climb();
        }
        return instance;
    }

    private final TalonSRX spool;
    private final DoubleSolenoid latch;
    private final DoubleSolenoid arm;

    public State DISABLED, STOWED, START_CLIMB, START_CLIMB_MANUAL, GRAB_MID, PULL_MID, PULL_MID_MANUAL,
            RESET, TUG_MID, AFTER_TUG, HOLD_MID, GRAB_HIGH, RELEASE_MID, RELEASE_MID_SAFELY,
            FINAL_HANG, OPEN_LOOP;

    private void extendHigh() {
        arm.set(Value.kForward);
    }

    private void retractHigh() {
        arm.set(Value.kReverse);
    }

    public void toggleHigh() {
        if(arm.get() == Value.kForward) arm.set(Value.kReverse);
        else if(arm.get() == Value.kReverse) arm.set(Value.kForward);
    }

    public void latch() {
        latch.set(Value.kForward);
    }

    public void unlatch() {
        latch.set(Value.kReverse);
    }

    public void toggleLatch() {
        if(latch.get() == Value.kForward) latch.set(Value.kReverse);
        else if(latch.get() == Value.kReverse) latch.set(Value.kForward);
    }

    private void unravel() {
        spool.set(TalonSRXControlMode.MotionMagic, ClimbConstants.unspoolDistance + offset);
    }

    private void reravel() {
        spool.set(TalonSRXControlMode.MotionMagic, -ClimbConstants.unspoolDistance + offset);
    }

    private void winch() {
        spool.set(TalonSRXControlMode.MotionMagic, ClimbConstants.winchDistance + offset);
    }

    private void tug() {
        spool.set(TalonSRXControlMode.MotionMagic, ClimbConstants.tugDistance + offset);
    }

    private void unravelManual() {
        spool.set(TalonSRXControlMode.PercentOutput, ClimbConstants.unspoolVolts / ClimbConstants.maxVolts);
    }

    private void winchManual() {
        spool.set(TalonSRXControlMode.PercentOutput, ClimbConstants.winchVolts / ClimbConstants.maxVolts);
    }

    private void stall() {
        spool.set(TalonSRXControlMode.PercentOutput, ClimbConstants.stallVolts / ClimbConstants.maxVolts);
        // spool.set(TalonSRXControlMode.Current, ClimbConstants.stallCurrent);
    }

    private void stop() {
        spool.set(TalonSRXControlMode.PercentOutput, 0.0);
    }

    private double offset;

    public void zero() {
        offset = spool.getSelectedSensorPosition();
    }

    public double getMeasurement() {
        return spool.getSelectedSensorPosition() - offset;
    }

    private Climb() {
        super("Climb");

        spool = cfg.leftSpool;
        arm = cfg.arm;
        latch = cfg.latch;

        //retractHigh();
        //latch();
        arm.set(Value.kReverse);
        latch.set(Value.kReverse);

        offset = 0.0;

        spool.selectProfileSlot(0, 0);
        spool.config_kP(0, ClimbConstants.kPu);
        spool.config_kI(0, ClimbConstants.kIu);
        spool.config_kD(0, ClimbConstants.kDu);
        spool.config_kF(0, ClimbConstants.kFu);
        
        spool.config_kP(1, ClimbConstants.kPu);
        spool.config_kI(1, ClimbConstants.kIu);
        spool.config_kD(1, ClimbConstants.kDu);
        spool.config_kF(1, ClimbConstants.kFu);

        spool.configMotionCruiseVelocity(ClimbConstants.cruiseVel);
        spool.configMotionAcceleration(ClimbConstants.acc);
        spool.configMotionSCurveStrength(ClimbConstants.smoothing);
        spool.setSelectedSensorPosition(0.0);

        // State
        DISABLED = new State("DISABLED",
                () -> {
                    // init
                    latch();
                    //retractHigh();
                    zero();
                }, () -> {
                    // periodic
                    stop();
                });
        STOWED = new State("STOWED", // name displayed to drivers
                () -> {
                    // init
                    stop();
                    latch();
                    //retractHigh();
                    zero();
                }, () -> {
                    // periodic
                    stop();
                });

        START_CLIMB = new State("START_CLIMB",
                () -> {
                    // init
                    unlatch();
                    extendHigh();
                    zero();
                    unravel();
                }, () -> {
                    // periodic
                });
        RESET = new State("RESET",
                () -> {
                    // init
                    unlatch();
                    retractHigh();
                    zero();
                    reravel();
                }, () -> {
                    // periodic
                });
        START_CLIMB_MANUAL = new State("START_CLIMB_MANUAL",
                () -> {
                    // init
                    unlatch();
                    //extendHigh();
                    zero();
                }, () -> {
                    // periodic
                    unravelManual();
                });

        GRAB_MID = new State("GRAB_MID",
                () -> {
                    // init
                    stop();
                    unlatch();
                    //extendHigh();
                    zero();
                }, () -> {
                    // periodic
                    stop();
                });
        TUG_MID = new State("TUG_MID",
                () -> {
                    // init
                    retractHigh();
                    unlatch();
                    zero();
                    tug();
                }, () -> {
                    // periodic
                });
        AFTER_TUG = new State("AFTER_TUG",
                () -> {
                    // init
                    //retractHigh();
                    unlatch();
                    zero();
                    tug();
                }, () -> {
                    // periodic
                });

        PULL_MID = new State("PULL_MID",
                () -> {
                    // init
                    stop();
                    retractHigh();
                    unlatch();
                    zero();
                    winch();
                }, () -> {
                    // periodic
                });
        PULL_MID_MANUAL = new State("PULL_MID_MANUAL",
                () -> {
                    // init
                    //retractHigh();
                    zero();
                    unlatch();
                }, () -> {
                    // periodic
                    winchManual();
                });
        HOLD_MID = new State("HOLD_MID",
                () -> {
                    // init
                    stop();
                    latch();
                    zero();
                    //retractHigh();
                }, () -> {
                    // periodic
                    stop();
                });

        GRAB_HIGH = new State("GRAB_HIGH",
                () -> {
                    // init
                    latch();
                    extendHigh();
                    zero();
                }, () -> {
                    // periodic
                    stop();
                });

        RELEASE_MID = new State("RELEASE_MID",
                () -> {
                    // init
                    unlatch();
                    stop();
                    extendHigh();
                    zero();
                }, () -> {
                    // periodic
                    stop();
                });

        RELEASE_MID_SAFELY = new State("RELEASE_MID_SAFELY",
                () -> {
                    // init
                    unlatch();
                    //extendHigh();
                    zero();
                }, () -> {
                    // periodic
                    stall();
                });
        FINAL_HANG = new State("FINAL_HANG",
                () -> {
                    // init
                    stop();
                    unlatch();
                    //extendHigh();
                    zero();
                }, () -> {
                    // periodic
                    stop();
                });
        OPEN_LOOP = new State("OPEN_LOOP",
                () -> {
                    // init
                    unlatch();
                    //retractHigh();
                    zero();
                }, () -> {
                    // periodic
                    
                });
        setCurrentState(DISABLED);

        //Logging
        AddDashboardEntryWrite("spool", 0.0, () -> {
            return getMeasurement();
        });
        AddDashboardEntryWrite("latch", false, () -> {
            return latch.get() != Value.kForward;
        });        
        AddDashboardEntryWrite("arm", false, () -> {
            return arm.get() == Value.kForward;
        });
        AddDashboardEntryState(DISABLED);
    }

    public void setOutput(double out, double otherOut) {
        spool.set(ControlMode.PercentOutput, out);
        //otherSpool.set(ControlMode.PercentOutput, -out);
    }

}

// (: yeet :)