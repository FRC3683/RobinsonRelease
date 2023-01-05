// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.config.DrivetrainConstants;
import frc.robot.config.InveyorConstants;
import frc.robot.config.ShooterConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Inveyor;
import frc.robot.subsystems.Shooter;

public class DavePath extends SequentialCommandGroup {

    protected Drivetrain drivetrain;
    protected Inveyor inveyor;
    protected Shooter shooter;

    Timer timer;
    OI oi;
    private ArrayList<DaveSegment> segs;

    /** Creates a new DavePath. */
    public DavePath() {
        // Use addRequirements() here to declare subsystem dependencies.
        drivetrain = Drivetrain.GetInstance();
        inveyor = Inveyor.GetInstance();
        shooter = Shooter.GetInstance();
        addRequirements(drivetrain, inveyor, shooter);
        oi = OI.getInstance();
        segs = new ArrayList<>();
        timer = new Timer();
    }

    @Override
    public void initialize() {
        super.initialize();
        drivetrain.setCurrentState(drivetrain.LGCL);
        int i = 0;
        while(i < segs.size() - 1){
            if(!segs.get(i).isDelay()){
                drivetrain.resetOdometry(segs.get(i).getInitialPose());
                timer.start();
                return;
            }
            i++;
        }
        drivetrain.resetOdometry(DaveSegment.defaultPose);
        timer.start();
    }

    public DavePath addSegment(String traj) {
        DaveSegment s = new DaveSegment(traj, getTotalTime());
        segs.add(s);
        addCommands(s.getCommand());
        return this;
    }    
    
    public DavePath addSegment(double time) {
        DaveSegment s = new DaveSegment(time, getTotalTime());
        segs.add(s);
        addCommands(s.getCommand());
        return this;
    }

    public double getTotalTime(){
        if(segs.isEmpty()) return 0.0;
        return segs.get(segs.size() - 1).getTotalTime();
    }

    public double time() {
        return timer.get();
    }

    public void reset() {
        timer.reset();
    }

    public boolean after(double seconds) {
        return timer.hasElapsed(seconds);
    }

    public boolean before(double seconds) {
        return !timer.hasElapsed(seconds);
    }

    public boolean between(double earlier, double later) {
        return after(earlier) && before(later);
    }

    public double alpha(double earlier, double later) {
        double t = timer.get();
        if (t <= earlier)
            return 0.0;
        if (t >= later)
            return 1.0;
        return MathUtils.unlerp(earlier, later, t);
    }

    protected void intake(){
        if(inveyor.ballAtGate() && inveyor.ballAtIntake()){
            inveyor.setCurrentState(inveyor.GRAB_BALL);
        }else if(inveyor.ballAtGate()){
            inveyor.setCurrentState(inveyor.INTAKE);
        } else {
            inveyor.setCurrentState(inveyor.INTAKE_FAST);
        }
    }

    protected void feed(){
        if(inveyor.INTAKE.isActive() || inveyor.INTAKE_FAST.isActive() || inveyor.GRAB_BALL.isActive() || inveyor.DROP_INTAKE.isActive()) inveyor.setCurrentState(inveyor.CHAMBER);      
        if(inveyor.CHAMBER.after(InveyorConstants.chamberTime_s)) inveyor.setCurrentState(inveyor.READY_TO_SHOOT);
        //else if (inveyor.READY_TO_SHOOT.after(InveyorConstants.autoShotDelay) && shooter.onAimlockTarget())inveyor.setCurrentState(inveyor.INTAKE_AND_FEED);
        else if (inveyor.READY_TO_SHOOT.isActive()){
            if(shooter.shouldShoot() &&  Math.abs(drivetrain.getAngularRate()) < DrivetrainConstants.baselock_rate && (drivetrain.AIMLOCK.after(InveyorConstants.autoShotDelay) || drivetrain.BASELOCK.isActive()) && shooter.SHOOTING.isActive()){
                inveyor.setCurrentState(inveyor.FEEDING_SHOOTER);
            } 
        }
    }

    protected void forcefeed(){
        if(inveyor.INTAKE.isActive() || inveyor.INTAKE_FAST.isActive() || inveyor.GRAB_BALL.isActive() || inveyor.DROP_INTAKE.isActive()) inveyor.setCurrentState(inveyor.CHAMBER);      
        if(inveyor.CHAMBER.after(InveyorConstants.chamberTime_s)) inveyor.setCurrentState(inveyor.READY_TO_SHOOT);
        //else if (inveyor.READY_TO_SHOOT.after(InveyorConstants.autoShotDelay) && shooter.onAimlockTarget())inveyor.setCurrentState(inveyor.INTAKE_AND_FEED);
        else if (inveyor.READY_TO_SHOOT.isActive()){
            inveyor.setCurrentState(inveyor.FEEDING_SHOOTER);
        }
    }

    protected void shoot(){
        double rpm = shooter.getTargetRPM();
        if(shooter.STOWED.isActive() || shooter.SPINDOWN.isActive()) shooter.setCurrentState(shooter.REVING);
        else if (shooter.REVING.isActive() /*&& shooter.atTargetRPM()*/ &&  shooter.getRPM() > rpm * ShooterConstants.fastRampToPreciseThreshold) {
            shooter.setCurrentState(shooter.SHOOTING);
        }
        shooter.setTargetRPM(rpm);
    }

    protected void shoot(double rpm){
        if(shooter.STOWED.isActive() || shooter.SPINDOWN.isActive()) shooter.setCurrentState(shooter.REVING);
        else if (shooter.REVING.isActive() /*&& shooter.atTargetRPM()*/ &&  shooter.getRPM() > rpm * ShooterConstants.fastRampToPreciseThreshold) {
            shooter.setCurrentState(shooter.SHOOTING);
        }
        shooter.setTargetRPM(rpm);
    }


    protected void aim(double turn){
        boolean aiming = drivetrain.STTBOLLS.isActive() || drivetrain.ASTTBOLLS.isActive() || drivetrain.AIMLOCK.isActive() || drivetrain.BASELOCK.isActive();
        if(!drivetrain.validTarget()){    
            drivetrain.setCurrentState(drivetrain.LGOL);
            drivetrain.splitStickDrive(0.0, turn);
        } else if(aiming && drivetrain.shouldAimlock()){
            drivetrain.setCurrentState(drivetrain.AIMLOCK);
        } else {
            drivetrain.setCurrentState(drivetrain.ASTTBOLLS);
        }
    }

    protected void driving(){
        drivetrain.setCurrentState(drivetrain.LGCL);  
        inveyor.setCurrentState(inveyor.DROP_INTAKE);
        shooter.setCurrentState(shooter.SPINDOWN); 
    }

    protected double segEnd(int i){
        return segs.get(i).getTotalTime();
    }

    @Override
    public boolean isFinished() {
        return super.isFinished() || oi.driverInput();
    }

    @Override
    public void end(boolean interuppted) {
        oi.rumbleDriverFor(0.2, 0.2, 2.0);
        drivetrain.setCurrentState(drivetrain.DISABLED);
        inveyor.setCurrentState(inveyor.DISABLED);
        shooter.setCurrentState(shooter.DISABLED);
        super.end(interuppted);
    }
}
