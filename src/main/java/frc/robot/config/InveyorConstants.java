// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config;

import edu.wpi.first.wpilibj.util.Color;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class InveyorConstants {
    // Specs
    public static final int conveyorPdpPort = 0;
    public static final double conveyorCurrentThreshold = 45.0;
    public static final double maxVolts = 12.0;
    public static final int currentLimit = 60;
    public static final double timeToFullOpen = 0.0;
    public static final double proxThreshold = 80.0;
    public static final double chamberTime_s = 0.15;
    public static final double chamberInterruptTime_s = 0.2;

    // Control
    public static final double kP = 0.6;
    public static final double kI = 0.002;
    public static final double kD = 0.01;
    public static final double kF = 0.1011208;
    public static final double kIz = 500;
    public static final double kPc = 0.8;
    public static final double kIc = 0.002;
    public static final double kDc = 0.01;
    public static final double kFc = 0.1011208;
    public static final double kIzc = 500;
    
    public static final double chamberSpeed = 10500.0;
    public static final double acc = 84000.0;
    public static final int smoothing = 0; //0-8


    //positive value = into robot/shooter
    //negative value = out of robot/shooter

    // Speeds
    public static final double intakePower = 0.70; 
    public static final double feedPower = 0.7; 
    public static final double feedSpeed = 5450;//7450; 
    public static final double conveyorPower = 0.55; 
    public static final double conveyorPowerLow = 0.45;
    public static final double conveyorPowerPassive = 0.0/12.0;
    public static final double conveyorReversePower = -0.4;
    public static final double conveyorChamberPower = -0.6;
    public static final double intakeReversePower = -0.45; // still intaking
    
    
    public static final double conveyorChamberTime_s = 0.2;
    public static final double gateBeamDebounceTime = 0.01;
    public static final double intakeBeamDebounceTime = 0.01;
    public static final double chamberDelay = 0.025;
    public static final double shotDelay = 0.075;
    public static final double autoShotDelay = 0.5;
    public static final double colourDiff = 0.10;

    // Positions
    public static final double chamberDist = -1750;
    public static final double feedDist = 0.0;
    
}
