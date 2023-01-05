// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Solenoid;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;

import frc.robot.config.InveyorConstants;

public class Inveyor extends DaveSubsystem {

    private static Inveyor instance;

    public static Inveyor GetInstance() {
        if (instance == null) {
            instance = new Inveyor();
        }
        return instance;
    }

    private PowerDistribution pdp;

    private VictorSPX intakeMotor;
    private Solenoid intakePiston;
    private TalonSRX conveyorMotor;
    private Solenoid gatePiston;
    private DigitalInput gateBeam;
    private DigitalInput intakeBeam;

    private Debouncer gateDebouncer;
    private Debouncer intakeDebouncer;

    public State DISABLED;
    public State FEEDING_SHOOTER;
    public State STOWED;
    public State CHAMBER;
    public State READY_TO_SHOOT;
    public State GRAB_BALL;
    public State INTAKE_FAST;
    public State INTAKE_AND_FEED;
    public State INTAKE;
    public State INTAKE_UP;
    public State STOPPED;
    public State HUMAN_FEED;
    public State REVERSE;
    public State SPIT;
    public State DROP_INTAKE;
    public State CLIMB;
    public State POST_SPIN;
    public State LOAD;
    public State LOAD_FAST;
    public State TOP_POOP;
    public State BOTTOM_POOP;
    

    public enum COLOUR{
        RED,
        BLUE,
        NONE
    }

    public enum INV{
        EMPTY,
        GOOD,
        BAD,
        GOOD_BAD,
        BAD_GOOD,
        TWO_GOOD,
        TWO_BAD,

    }

    private COLOUR alliance = COLOUR.NONE;
    private COLOUR opps = COLOUR.NONE;
    public INV inv;

    public void setAllianceAsRed(){
        alliance = COLOUR.RED;
        opps = COLOUR.BLUE;
    }

    public void setAllianceAsBlue(){
        alliance = COLOUR.BLUE;
        opps = COLOUR.RED;
    }

    public void setAllianceAsNone(){
        alliance = COLOUR.NONE;
        opps = COLOUR.NONE;
    }

    // if there is no ball, put a ball in to the first slot, if there is already a
    // ball, put the new ball in the second slot, if there are 2 balls, then don't
    // change the array
    public void addBall(COLOUR ball) {
        switch(inv){
            case EMPTY:{
                if(alliance == COLOUR.NONE || ball == alliance) inv = INV.GOOD;
                else if(ball == opps) inv = INV.BAD;
            } break;
            case GOOD:{
                if(alliance == COLOUR.NONE || ball == alliance) inv = INV.TWO_GOOD;
                else if(ball == opps) inv = INV.GOOD_BAD;
            } break;
            case BAD:{
                if(alliance == COLOUR.NONE || ball == alliance) inv = INV.BAD_GOOD;
                else if(ball == opps) inv = INV.TWO_BAD;
            } break;
            case GOOD_BAD:{    
            } break;
            case BAD_GOOD:{
            } break;
            case TWO_GOOD:{
                if(alliance == COLOUR.NONE || ball == alliance) inv = INV.TWO_GOOD;
                else if(ball == opps) inv = INV.GOOD_BAD;
            } break;
            case TWO_BAD:{
                if(alliance == COLOUR.NONE || ball == alliance) inv = INV.BAD_GOOD;
                else if(ball == opps) inv = INV.TWO_BAD;
            } break;
        }
    }

    // updates the stored array when a ball is removed, either from the top or the
    // bottom
    // action: TOP = out of shooter, BOTTOM = out of intake
    public void shootBall() {
        switch(inv){
            case EMPTY:{
                inv = INV.EMPTY;
            } break;
            case GOOD:{
                inv = INV.EMPTY;
            } break;
            case BAD:{
                inv = INV.EMPTY;
            } break;
            case GOOD_BAD:{
                inv = INV.BAD;
            } break;
            case BAD_GOOD:{
                inv = INV.GOOD;
            } break;
            case TWO_GOOD:{
                inv = INV.GOOD;
            } break;
            case TWO_BAD:{
                inv = INV.BAD;
            } break;
        }
    };

    public void spitBall() {
        switch(inv){
            case EMPTY:{
                inv = INV.EMPTY;
            } break;
            case GOOD:{
                inv = INV.EMPTY;
            } break;
            case BAD:{
                inv = INV.EMPTY;
            } break;
            case GOOD_BAD:{
                inv = INV.GOOD;
            } break;
            case BAD_GOOD:{
                inv = INV.BAD;
            } break;
            case TWO_GOOD:{
                inv = INV.GOOD;
            } break;
            case TWO_BAD:{
                inv = INV.BAD;
            } break;
        }
    };

    // return TOP if you should poop out of the shooter, return BOTTOM if you should
    // poop out the intake, return hold your poop if you do not need to poop
    // If the robot has no balls it's a coward lol
    public static String poopCorrect(String[] balls) {

        // if all the balls are not the enemies
        if (balls[0] != "WRONG" && balls[1] != "WRONG") {
            return "HOLD YOUR POOP";
        }

        // returns top or bottom depending on if the enemy ball is next to the shooter
        // or the intake
        return balls[0] == "WRONG" ? "TOP" : "BOTTOM";
    }


    // related to colour sensor (poop the wrong coloured ball out cause its shit)

    // Poop_Ball discontinued after change to CAD
    // public State POOP_BALL = new State("POOP_BALL",
    // () -> {
    // // init
    // }, () -> {
    // // periodic
    // setOutput(Constants.intakeMotorReverseOutput,
    // Constants.intakeConveyorReverseOutput);
    // }
    // );

    public boolean chambered(){
        return false; //beam.get();
    }


    /** Creates a new Inveyor. */
    private Inveyor() {
        super("Inveyor");

        intakeMotor = cfg.intakeRight;
        conveyorMotor = cfg.conveyor;

        conveyorMotor.selectProfileSlot(0, 0);
        conveyorMotor.config_kP(0, InveyorConstants.kP);
        conveyorMotor.config_kI(0, InveyorConstants.kI);
        conveyorMotor.config_kD(0, InveyorConstants.kD);
        conveyorMotor.config_kF(0, InveyorConstants.kF);
        conveyorMotor.config_IntegralZone(0, InveyorConstants.kIz);
        conveyorMotor.config_kP(1, InveyorConstants.kPc);
        conveyorMotor.config_kI(1, InveyorConstants.kIc);
        conveyorMotor.config_kD(1, InveyorConstants.kDc);
        conveyorMotor.config_kF(1, InveyorConstants.kFc);
        conveyorMotor.config_IntegralZone(1, InveyorConstants.kIzc);

        conveyorMotor.configMotionCruiseVelocity(InveyorConstants.chamberSpeed);
        conveyorMotor.configMotionAcceleration(InveyorConstants.acc);
        conveyorMotor.configMotionSCurveStrength(InveyorConstants.smoothing);
        conveyorMotor.setSelectedSensorPosition(0.0);

        gateBeam = cfg.gateBeam;
        intakeBeam = cfg.intakeBeam;

        intakePiston = cfg.intakePiston;
        gatePiston = cfg.gatePiston;
        //beam = cfg.beam;
        gateDebouncer = new Debouncer(InveyorConstants.gateBeamDebounceTime, DebounceType.kBoth);
        intakeDebouncer = new Debouncer(InveyorConstants.intakeBeamDebounceTime, DebounceType.kBoth);

        inv = INV.EMPTY;
        setAllianceAsNone();

        // States
        DISABLED = new State("DISABLED", // name displayed to driver station
                () -> {
                    // init
                    intakeUp();
                    gateClosed();
                    zero();
                }, () -> {
                    // periodic
                    runIntake(0.0);
                    runConveyor(0.0);
                });
        FEEDING_SHOOTER = new State("FEEDING_SHOOTER", // name displayed to driver station
                () -> {
                    // init
                    gateOpen();
                    zero();
                    conveyorMotor.selectProfileSlot(0, 0);
                }, () -> {
                    // periodic
                    
                    runIntake(0.0);
                    //runConveyor(InveyorConstants.feedPower);
                    velocityConveyor(InveyorConstants.feedSpeed);
                    //positionConveyor(InveyorConstants.feedDist);
                });
        TOP_POOP = new State("TOP_POOP", // name displayed to driver
                //used for BAD and TWO_BAD state
                () -> {
                    // init
                    //intakeUp();
                }, () -> {
                    // periodic
                    runIntake(0.0);
                    runConveyor(0.0);
                });

        BOTTOM_POOP = new State("BOTTOM_POOP", // name displayed to driver station
                //used for GOOD_BAD state
                () -> {
                    // BRI'ISH INNIT???
                    //intakeDown();
                }, () -> {
                    // periodic
                    runIntake(0.0);
                    runConveyor(0.0);
                });

        STOWED = new State("STOWED", // name displayed to driver station
                () -> {
                    // init
                    gateClosed();
                    intakeUp();
                    //conveyorMotor.setSelectedSensorPosition(0.0);
                    zero();
                }, () -> {
                    // periodic
                    //runIntake(0.0);
                    //runConveyor(InveyorConstants.conveyorPowerPassive);
                    
                    if(ballAtGate()){                        
                        runIntake(0.0);
                        runConveyor(InveyorConstants.conveyorPowerPassive);
                    } else if(inv == INV.GOOD){                        
                        runIntake(0.0);
                        runConveyor(InveyorConstants.conveyorPowerLow);
                    } else {                        
                        runIntake(0.0);
                        runConveyor(0.0);
                    }
                });

        READY_TO_SHOOT = new State("READY_TO_SHOOT", // name displayed to driver station
                () -> {
                    // init
                    gateOpen();
                    zero();
                }, () -> {
                    // periodic
                    runIntake(0.0);
                    runConveyor(0.0);
                });
        STOPPED = new State("STOPPED", // name displayed to driver station
                () -> {
                    // init
                    gateClosed();
                    zero();
                }, () -> {
                    // periodic
                    runIntake(0.0);
                    runConveyor(0.0);
                });
        GRAB_BALL = new State("GRAB_BALL",
                () -> {
                    // init
                    gateClosed();
                    intakeDown();
                    zero();

                }, () -> {
                    // periodic
                            
                    runIntake(InveyorConstants.intakePower);
                    runConveyor(0.0);
                });
        INTAKE_FAST = new State("INTAKE_FAST",
                () -> {
                    // init
                    gateClosed();
                    intakeDown();
                    zero();
                }, () -> {
                    // periodic

                    runIntake(InveyorConstants.intakePower);
                    runConveyor(InveyorConstants.conveyorPower);
                    //positionConveyor(InveyorConstants.feedDist);
                });
        INTAKE_AND_FEED = new State("INTAKE_AND_FEED",
                () -> {
                    // init
                    gateOpen();
                    intakeDown();
                    conveyorMotor.selectProfileSlot(0, 0);
                    zero();
                }, () -> {
                    // periodic

                    runIntake(InveyorConstants.intakePower);
                    velocityConveyor(InveyorConstants.feedSpeed);
                    //positionConveyor(InveyorConstants.feedDist);
                });
        INTAKE = new State("INTAKE",
                () -> {
                    // init
                    gateClosed();
                    intakeDown();
                    zero();
                }, () -> {
                    // periodic

                    runIntake(InveyorConstants.intakePower);
                    runConveyor(InveyorConstants.conveyorPowerLow);
                });
        POST_SPIN = new State("POST_SPIN",
                () -> {
                    // init
                    gateClosed();
                    intakeUp();
                    zero();
                }, () -> {
                    // periodic

                    runIntake(InveyorConstants.intakePower);
                    runConveyor(InveyorConstants.conveyorPowerLow);
                });
        LOAD = new State("LOAD",
                () -> {
                    // init
                    gateClosed();
                    intakeUp();
                    zero();
                }, () -> {
                    // periodic

                    runIntake(InveyorConstants.intakePower);
                    runConveyor(InveyorConstants.conveyorPowerLow);
                });
        LOAD_FAST = new State("LOAD_FAST",
                () -> {
                    // init
                    gateClosed();
                    intakeUp();
                    zero();
                }, () -> {
                    // periodic

                    runIntake(InveyorConstants.intakePower);
                    runConveyor(InveyorConstants.conveyorPower);
                });
        INTAKE_UP = new State("INTAKE_UP",
                () -> {
                    // init
                    gateClosed();
                    intakeUp();
                }, () -> {
                    // periodic

                    runIntake(0.0);
                    runConveyor(0.0);
                });
        HUMAN_FEED = new State("HUMAN_FEED",
                () -> {
                    // init
                    //intakeUp();
                }, () -> {
                    // periodic
                    runIntake(InveyorConstants.intakePower);
                    runConveyor(InveyorConstants.conveyorPower);
                });

        // poops any ball currently stored (might not be shit) //!swear
        REVERSE = new State("REVERSE",
                () -> {
                    // init
                    gateOpen();
                    zero();
                }, () -> {
                    // periodic
                    if(getIntakeDown()){
                        runIntake(InveyorConstants.intakeReversePower);
                    } else {
                        runIntake(InveyorConstants.intakeReversePower);
                    }
                    runConveyor(InveyorConstants.conveyorReversePower);
                });  
                
        SPIT = new State("SPIT",
                () -> {
                    // init
                    gateOpen();
                    intakeDown();
                    zero();
                }, () -> {
                    // periodic
                    runIntake(InveyorConstants.intakeReversePower);
                    runConveyor(0.0);
                }); 
        
        DROP_INTAKE = new State("DROP_INTAKE",
                () -> {
                    // init
                    gateClosed();
                    intakeDown();
                }, () -> {
                    // periodic
                    runIntake(0.0);
                    runConveyor(0.0);
                });
        CLIMB = new State("CLIMB",
                () -> {
                    // init
                    gateClosed();
                    intakeDown();
                }, () -> {
                    // periodic
                    runIntake(0.0);
                    runConveyor(0.0);
                });
        CHAMBER = new State("CHAMBER",
                () -> {
                    // init
                    gateOpen();
                    conveyorMotor.selectProfileSlot(1, 0);
                    zero();
                }, () -> {
                    // periodic
                    runIntake(0.0);
                    //velocityConveyor(-InveyorConstants.chamberSpeed);
                    positionConveyor(InveyorConstants.chamberDist);
                });
        setCurrentState(DISABLED);

        // Logging
        /*
        AddDashboardEntryWrite("R", 0.0, () -> {
            return colour.getRed();
        });
        AddDashboardEntryWrite("G", 0.0, () -> {
            return colour.getGreen();
        });
        AddDashboardEntryWrite("B", 0.0, () -> {
            return colour.getBlue();
        });
        AddDashboardEntryWrite("IR", 0.0, () -> {
            return colour.getIR();
        });
        AddDashboardEntryWrite("Prox", 0.0, () -> {
            return colour.getProximity();
        });
        */
        AddDashboardEntryWrite("Intake out", false, () -> {
            return intakePiston.get();
        });
        AddDashboardEntryWrite("Gate", false, () -> {
            return gatePiston.get();
        });
        AddDashboardEntryWrite("Conv Speed", 0.0, () -> {
            return conveyorSpeed();
        });

        AddDashboardEntryWrite("Conv Pos", 0.0, () -> {
            return conveyorPos();
        });
           
        AddDashboardEntryWrite("ball at gate", false, () -> {
            return ballAtGate();
        }); 
               
        AddDashboardEntryWrite("ball at intake", false, () -> {
            return ballAtIntake();
        });
               
        AddDashboardEntryWrite("Inventory", INV.EMPTY.toString(), () ->{
            return this.inv.toString();
        });
        AddDashboardEntryState(DISABLED);
    }

    private double offset;

    public void zero() {
        offset = conveyorMotor.getSelectedSensorPosition();
    }

    public double getConveyorPos() {
        return conveyorMotor.getSelectedSensorPosition() - offset;
    }

    private void intakeDown(){
        intakePiston.set(true);
    }

    private void intakeUp(){
        intakePiston.set(false);
    }

    private boolean getIntakeDown(){
        return intakePiston.get();
    }

    public void gateOpen(){
        gatePiston.set(false);
    }

    public void gateClosed(){
        gatePiston.set(true); //true
    }

    public void toggleGate(){
        gatePiston.set(!gatePiston.get());
    }

    public void toggleIntake(){
        intakePiston.set(!intakePiston.get());
    }

    private void runIntake(double power) {
        intakeMotor.set(ControlMode.PercentOutput, power);
    }

    private void runConveyor(double power) {
        conveyorMotor.set(ControlMode.PercentOutput, power);
    }

    private void positionConveyor(double position){
        conveyorMotor.set(TalonSRXControlMode.MotionMagic, position + offset);
    }

    private void velocityConveyor(double velocity){
        conveyorMotor.set(TalonSRXControlMode.Velocity, velocity);
    }

    private double conveyorSpeed(){
        return conveyorMotor.getSelectedSensorVelocity();
    }

    private double conveyorPos(){
        return conveyorMotor.getSelectedSensorPosition();
    }


    public double conveyorCurrent(){
        return conveyorMotor.getSupplyCurrent();
    }

    
    public boolean ballAtGate(){
        return gateBeam.get();
    }

    public boolean ballAtIntake(){
        return intakeBeam.get();
    }


    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        super.periodic();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
