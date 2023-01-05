// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climb;
import frc.robot.utils.OI;

public class Rewinch extends CommandBase {

    private Climb climb;
    private OI oi;

    /** Creates a new Rewinch. */
    public Rewinch() {
        // Use addRequirements() here to declare subsystem dependencies.
        climb = Climb.GetInstance();
        oi = OI.getInstance();
        addRequirements(climb);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        climb.setCurrentState(climb.OPEN_LOOP);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(climb.OPEN_LOOP.isActive()){
            climb.setOutput(-0.1, -0.1);
            //climb.setOutput(-oi.getYLeftDriver()*0.25, -oi.getYRightDriver()*0.25);
            //if(oi.getStartButtonPressedDriver()) climb.toggleLatch();
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
