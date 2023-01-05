// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.ShooterConstants;
import frc.robot.subsystems.Eye;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.MathUtils;
import frc.robot.utils.OI;

public class ShooterTeleop extends CommandBase {

    private Shooter shooter;
    private OI oi;

    /** Creates a new ShooterTeleop. */
    public ShooterTeleop() {
        // Use addRequirements() here to declare subsystem dependencies.
        shooter = Shooter.GetInstance();
        oi = OI.getInstance();
        addRequirements(shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        shooter.setCurrentState(shooter.STOWED);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (!shooter.DISABLED.isActive()) {
            if(oi.getYButtonPressedOperator()){
                shooter.toggleHood();
            }

            boolean shooting = oi.getXButtonDriver() || oi.getAButtonDriver() || oi.getBButtonDriver()
                    || oi.getYButtonDriver() || oi.getRightTriggerDriver() > 0.5;

            double targetRPM = 0.0;

            if (oi.getXButtonDriver()) {
                targetRPM = ShooterConstants.rpm_prerev;
            } else if (oi.getAButtonDriver()) {
                targetRPM = ShooterConstants.rpm_freethrow;
            } else if (oi.getBButtonDriver()) {
                targetRPM = ShooterConstants.rpm_ideal;
            } else if (oi.getYButtonDriver()) {
                targetRPM = ShooterConstants.rpm_far;
            } else if (oi.getRightTriggerDriver() > 0.5) {
                targetRPM = shooter.getTargetRPM();
                shooter.timeLogs.append(Timer.getMatchTime());
                shooter.shotDistLogs.append(Eye.getInstance().getDistance());
                shooter.shotRPMLogs.append(targetRPM);
            }

            if (shooting) {
                if(shooter.STOWED.isActive() || shooter.SPINDOWN.isActive()) shooter.setCurrentState(shooter.REVING);
                else if (shooter.REVING.isActive() /*&& shooter.atTargetRPM()*/ &&  shooter.getRPM() > targetRPM * ShooterConstants.fastRampToPreciseThreshold) {
                    shooter.setCurrentState(shooter.SHOOTING);
                }
                shooter.setTargetRPM(targetRPM);
            } else {
                shooter.setCurrentState(shooter.SPINDOWN);
            }
        } else {
            oi.getYButtonPressedOperator();
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.setCurrentState(shooter.DISABLED);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
