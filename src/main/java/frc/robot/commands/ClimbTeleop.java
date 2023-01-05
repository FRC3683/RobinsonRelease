// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.swing.plaf.TreeUI;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.ClimbConstants;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Inveyor;
import frc.robot.subsystems.DaveSubsystem.State;
import frc.robot.utils.MathUtils;
import frc.robot.utils.OI;

public class ClimbTeleop extends CommandBase {

    private boolean resetting = false;

    private Climb climb;
    private OI oi;

    /** Creates a new ClimbTeleop. */
    public ClimbTeleop() {
        // Use addRequirements() here to declare subsystem dependencies.
        climb = Climb.GetInstance();
        oi = OI.getInstance();
        addRequirements(climb);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (resetting)
            climb.setCurrentState(climb.OPEN_LOOP);
        else
            climb.setCurrentState(climb.STOWED);
        climb.zero();
    }

    /*
     * IDEA:
     * unlatch
     * unravel both spools (slave em) while intitating high bar solenoid
     * button press: winch spools a certain amount and untrigger high solenoid
     * latch
     * fire high solenoid
     * wait for button: unlatch
     * if that does work: unlatch and stall against the fall
     * 
     * stall at 4 volts
     * 
     * Called every time the scheduler runs while the command is scheduled.
     */
    @Override
    public void execute() {
        if (!climb.DISABLED.isActive()) {

            if (climb.OPEN_LOOP.isActive()) {
                climb.setOutput(-oi.getYLeftOperator() * 0.25, -oi.getYRightOperator() * 0.25);
                if (oi.getRightBumperPressedOperator())
                    climb.toggleLatch();
                if (oi.getAButtonPressedOperator())
                    climb.setCurrentState(climb.STOWED);
                if (oi.getLeftBumperPressedOperator())
                    climb.toggleHigh();
                if (oi.getBButtonPressedOperator())
                    climb.zero();
            } else if (climb.STOWED.isActive()) {
                if (oi.getAButtonPressedOperator())
                    climb.setCurrentState(climb.OPEN_LOOP);
                if (oi.getRightBumperPressedOperator())
                    climb.toggleLatch();
                if (oi.getLeftBumperPressedOperator())
                    climb.toggleHigh();
                if (oi.getBButtonPressedOperator())
                    climb.zero();
                if (oi.getDPadUpDriver()) {
                    climb.setCurrentState(climb.START_CLIMB);
                }
            } else if (climb.START_CLIMB.isActive()
                    && MathUtils.closeEnoughPercent(climb.getMeasurement(), ClimbConstants.unspoolDistance, 0.025)) {
                climb.setCurrentState(climb.GRAB_MID);
            } else if (climb.START_CLIMB_MANUAL.isActive()) {
                if (oi.getDPadUpOperator()) {
                    climb.setCurrentState(climb.START_CLIMB_MANUAL);
                } else
                    climb.setCurrentState(climb.STOWED);
            } else if (climb.GRAB_MID.isActive()) {
                if (oi.getDPadLeftDriver()) {
                    climb.setCurrentState(climb.TUG_MID);
                }else if (oi.getDPadDownDriver()) {
                    climb.setCurrentState(climb.RESET);
                }
            } else if (climb.RESET.isActive()
                    && MathUtils.closeEnoughPercent(climb.getMeasurement(), -ClimbConstants.unspoolDistance, 0.025)) {
                climb.setCurrentState(climb.STOWED);
            } else if (climb.TUG_MID.isActive() && oi.operatorInput()) {
                climb.setCurrentState(climb.AFTER_TUG);
            } else if (climb.AFTER_TUG.isActive()) {
                climb.setOutput(-oi.getYLeftOperator(), -oi.getYRightOperator());
                if (oi.getRightBumperPressedOperator())
                    climb.toggleLatch();
                if (oi.getAButtonPressedOperator())
                    climb.setCurrentState(climb.STOWED);
                if (oi.getLeftBumperPressedOperator()) {

                    climb.toggleHigh();
                    climb.latch();
                }
                if (oi.getBButtonPressedOperator())
                    climb.zero();
            } else if (climb.PULL_MID.isActive()) {
                if (oi.getDPadRightDriver()
                        || MathUtils.closeEnoughPercent(climb.getMeasurement(), ClimbConstants.winchDistance, 0.065)) {
                    climb.setCurrentState(climb.GRAB_HIGH);
                }
            } else if (climb.GRAB_HIGH.isActive() && oi.getDPadUpDriver()) {
                climb.setCurrentState(climb.RELEASE_MID);
            } else if (climb.RELEASE_MID.isActive()) {
                climb.setCurrentState(climb.RELEASE_MID);
            } else {
                // climb.setCurrentState(climb.STOWED);
            }
        } else

        {
            oi.getRightBumperPressedOperator();
            oi.getAButtonPressedOperator();
            oi.getLeftBumperPressedOperator();
            if (oi.getBButtonPressedOperator())
                climb.zero();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        climb.setCurrentState(climb.DISABLED);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
