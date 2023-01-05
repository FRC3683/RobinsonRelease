// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.utils.DavePath;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAuto extends DavePath {
    
    /** Creates a new QualAuto. */
    public TestAuto() {
        super();
        addSegment(0.5).addSegment("test");
    }

    @Override
    public void initialize() {
        super.initialize();
        inveyor.setCurrentState(inveyor.STOWED);
        shooter.setCurrentState(shooter.STOWED);
    }

    @Override
    public void execute(){
        super.execute();
    }

}
