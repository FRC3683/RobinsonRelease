// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.utils.DavePath;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveForward extends DavePath {
    
    /** Creates a new QualAuto. */
    public DriveForward() {
        super();
        addSegment(0.5).addSegment("fwd").addSegment(3.9);
    }

    @Override
    public void initialize() {
        super.initialize();
        drivetrain.setRamsete(false);
        inveyor.setCurrentState(inveyor.STOWED);
        shooter.setCurrentState(shooter.STOWED);
    }

    @Override
    public void execute(){
        super.execute();

        if(between(0.0, segEnd(1))){
            intake();
        }

        if(between(segEnd(1) + 0.9, segEnd(2))){
            feed();
        }

        if(between(segEnd(1), segEnd(2))){            
            shoot();
        }

        if(between(segEnd(1), segEnd(2))){
            aim(0.6);
        }

    }

}
