// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Eye;
import frc.robot.utils.OI;

public class DriveTeleop extends CommandBase {

    private Drivetrain drivetrain;
    private Eye eye;
    private final OI oi;

    /** Creates a new DriveTeleop. */
    public DriveTeleop() {
        // Use addRequirements() here to declare subsystem dependencies.
        drivetrain = Drivetrain.GetInstance();
        eye = Eye.getInstance();
        oi = OI.getInstance();

        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        drivetrain.setCurrentState(drivetrain.LGOL);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(!drivetrain.DISABLED.isActive()){
            drivetrain.splitStickDrive(-oi.getYLeftDriver(), oi.getXRightDriver());
            if(oi.getRightTriggerDriver() > 0.5){
                boolean aiming = drivetrain.STTBOLLS.isActive() || drivetrain.AIMLOCK.isActive() || drivetrain.BASELOCK.isActive();
                if(aiming && drivetrain.shouldAimlock()){
                    drivetrain.setCurrentState(drivetrain.AIMLOCK);
                } else {
                    drivetrain.setCurrentState(drivetrain.STTBOLLS);
                }
            } else if(oi.getRightBumperDriver()){
                drivetrain.setCurrentState(drivetrain.HGOL);
            } else if(oi.getDPadUpDriver()){
                drivetrain.setCurrentState(drivetrain.CLIMB);
            } else if(drivetrain.CLIMB.isActive() && !oi.getDPadDownDriver()){
                drivetrain.setCurrentState(drivetrain.CLIMB);
            } else {
                drivetrain.setCurrentState(drivetrain.LGOL);
            }

        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.splitStickDrive(0, 0);
        drivetrain.setCurrentState(drivetrain.DISABLED);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
