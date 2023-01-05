// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class EyeConstants {
    public static final double yawKP = 0.03;
    public static final double yawKI = 0.0;
    public static final double yawKD = 0.075;
    public static final double yawTolerance = 2;

    public static final double desiredTargetYaw = 0.0;
    public static final double throttle = 0.5;
    public static final double headingTolerance = 10; // TODO fix this value
    public static final double aimlockTolerance = 3.2; // TODO fix this value
    public static final double baselockTolerance = 0.5; // TODO fix this value

    public static final double limelightMountAngle = 22.5;
    public static final double goalHeight = 104 ;
    public static final double limelightMountHeight = 44.9;
}
