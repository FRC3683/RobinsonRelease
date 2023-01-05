// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.InvertType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.DrivetrainConstants;
import frc.robot.config.InveyorConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Inveyor;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.MathUtils;
import frc.robot.utils.OI;

public class InveyorTeleop extends CommandBase {

    private Inveyor inveyor;
    private Drivetrain drivetrain;
    private Shooter shooter;
    private OI oi;
    private boolean dontPost = false;

    /** Creates a new InveyorTeleop. */
    public InveyorTeleop() {
        // Use addRequirements() here to declare subsystem dependencies.
        inveyor = Inveyor.GetInstance();
        shooter = Shooter.GetInstance();
        drivetrain = Drivetrain.GetInstance();
        oi = OI.getInstance();
        addRequirements(inveyor);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        inveyor.setCurrentState(inveyor.STOWED);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (!inveyor.DISABLED.isActive()) {

            boolean aiming = oi.getRightTriggerDriver() > 0.5;
            boolean shooting = oi.getXButtonDriver() || oi.getAButtonDriver() || oi.getBButtonDriver()
                    || oi.getYButtonDriver() || aiming;
            if (oi.getStartButtonPressedDriver()) {
                inveyor.toggleIntake();
            }
            if(oi.getDPadUpDriver()){
                inveyor.setCurrentState(inveyor.CLIMB);
            } else if(oi.getDPadDownDriver()){                
                inveyor.setCurrentState(inveyor.STOWED);
            }
            if(shooting && oi.getLeftTriggerDriver() > 0.5){
                dontPost = true;
                inveyor.inv = Inveyor.INV.EMPTY;
                inveyor.setCurrentState(inveyor.INTAKE_AND_FEED);
            } else if (oi.getLeftTriggerDriver() > 0.5) {
                dontPost = false;
                if(oi.getLeftBumperDriver()){
                    inveyor.setCurrentState(inveyor.SPIT);
                }else if(inveyor.ballAtGate() && inveyor.ballAtIntake()){
                    inveyor.setCurrentState(inveyor.GRAB_BALL);
                }else if(inveyor.ballAtGate()){
                    inveyor.setCurrentState(inveyor.INTAKE);
                } else {
                    inveyor.setCurrentState(inveyor.INTAKE_FAST);
                }
            } else if (oi.getLeftBumperDriver()) {
                dontPost = false;
                inveyor.inv = Inveyor.INV.EMPTY;
                inveyor.setCurrentState(inveyor.REVERSE);
            } else if (shooting) {
                dontPost = true;
                inveyor.inv = Inveyor.INV.EMPTY;
                if(inveyor.STOWED.isActive() || inveyor.POST_SPIN.isActive()){
                    inveyor.setCurrentState(inveyor.CHAMBER);
                }
                if(inveyor.CHAMBER.isActive() && MathUtils.closeEnough(inveyor.getConveyorPos(), InveyorConstants.chamberDist, 250)) inveyor.setCurrentState(inveyor.READY_TO_SHOOT);
                else if (inveyor.READY_TO_SHOOT.isActive()){
                    if(aiming){                        
                        if(shooter.shouldShoot() && Math.abs(drivetrain.getAngularRate()) < DrivetrainConstants.baselock_rate && (drivetrain.AIMLOCK.after(InveyorConstants.shotDelay) || drivetrain.BASELOCK.isActive()) && shooter.SHOOTING.isActive()){
                            inveyor.setCurrentState(inveyor.FEEDING_SHOOTER);
                        } 
                    } else {
                        if(shooter.SHOOTING.isActive()){
                            inveyor.setCurrentState(inveyor.FEEDING_SHOOTER);
                        } 
                    }
                };
            } else if (oi.getBackButtonDriver()) {
                dontPost = false;
                if(inveyor.ballAtGate() && inveyor.ballAtIntake()){
                    inveyor.setCurrentState(inveyor.STOPPED);
                }else if(inveyor.ballAtGate()){
                    inveyor.setCurrentState(inveyor.LOAD);
                } else {
                    inveyor.setCurrentState(inveyor.LOAD_FAST);
                }
            } else {
                // if(inveyor.CHAMBER.after(InveyorConstants.chamberTime_s))
                // inveyor.setCurrentState(inveyor.STOWED);
                // if(inveyor.STOWED.isActive() && inveyor.chambered())
                // inveyor.setCurrentState(inveyor.CHAMBER);
                if(!inveyor.CLIMB.isActive() && !inveyor.STOWED.isActive())inveyor.setCurrentState(inveyor.POST_SPIN);
                if(dontPost || inveyor.POST_SPIN.after(2.0) || (inveyor.ballAtIntake() && inveyor.ballAtGate())){
                    dontPost = false;
                    inveyor.setCurrentState(inveyor.STOWED);
                }
            }
        } else {
            oi.getStartButtonPressedDriver();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        inveyor.setCurrentState(inveyor.DISABLED);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
