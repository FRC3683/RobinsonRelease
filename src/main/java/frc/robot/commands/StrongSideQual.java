// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.config.InveyorConstants;
import frc.robot.config.ShooterConstants;
import frc.robot.utils.DavePath;
import frc.robot.utils.MathUtils;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StrongSideQual extends DavePath {


    /** Creates a new QualAuto. */
    public StrongSideQual() {
        super();
        addSegment(0.2).addSegment("qual_leg1").addSegment(4.0);
        addSegment("qual_leg2").addSegment("qual_leg3").addSegment(4.0);
        //addSegment("qual_leg4").addSegment("qual_leg5");
        //addSegment(0.5).addSegment("qual_leg3");
        //addSegment(1.5).addSegment("qual_leg4");
        //addSegment(15.0);
    }

    @Override
    public void initialize() {
        super.initialize();
        inveyor.setCurrentState(inveyor.STOWED);
        shooter.setCurrentState(shooter.SPINDOWN);
    }

    @Override
    public void execute(){
        super.execute();


        if(between(0.0, segEnd(1))){
            intake();
        }

        if(between(segEnd(1) + 0.8, segEnd(2))){
            feed();
        }

        if(between(segEnd(1), segEnd(2))){            
            shoot();
        }

        if(between(segEnd(1), segEnd(2))){
            aim(0.7);
        }
        
        if(between(segEnd(2), segEnd(2) + 0.1)){          
            driving();
        }

        if(between(segEnd(3) + 1.0, segEnd(4))){
            intake();
        }

        if(between(segEnd(4) + 0.5, segEnd(5))){
            feed();
        }

        if(between(segEnd(4), segEnd(5))){
            aim(-0.7);
        }

        if(between(segEnd(4), segEnd(5))){
            shoot();
        }

        /*if(between(segEnd(5) + 0.5, segEnd(6))){
            intake();
        }

        if(between(segEnd(5), segEnd(6))){
            driving();
        }*/

    }

    @Override
    public boolean isFinished(){
        return super.isFinished() || after(15.0);
    }

}
