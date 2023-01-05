// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.paths;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.config.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

/** Add your docs here. */
public class DaveTrajectory {
    private static DaveTrajectory instance;

    public static DaveTrajectory GetInstance() {
        if (instance == null) {
            instance = new DaveTrajectory();
        }
        return instance;
    }
        
    private static Pose2d offset = new Pose2d(0.427, 0.302, Rotation2d.fromDegrees(0));

    public final DifferentialDriveVoltageConstraint autoVoltageConstraint;
    public final TrajectoryConfig forwardConfig;
    public final TrajectoryConfig reverseConfig;
    
    private final Drivetrain drivetrain;

    private DaveTrajectory(){
        drivetrain = Drivetrain.GetInstance();

        autoVoltageConstraint = new DifferentialDriveVoltageConstraint(drivetrain.ff, 
                                drivetrain.diffyKin, 
                                DrivetrainConstants.autoMaxVoltage_V);

        forwardConfig = forwardConfig();
        reverseConfig = reverseConfig();
    }

    public TrajectoryConfig forwardConfig(){
        return new TrajectoryConfig(
            DrivetrainConstants.autoMaxVelocity_m_s,
            DrivetrainConstants.autoMaxAcceleration_m_s2)
            .setKinematics(drivetrain.diffyKin)
            .addConstraint(autoVoltageConstraint);
    }

    public TrajectoryConfig reverseConfig(){
        return new TrajectoryConfig(
            DrivetrainConstants.autoMaxVelocity_m_s,
            DrivetrainConstants.autoMaxAcceleration_m_s2)
            .setKinematics(drivetrain.diffyKin)
            .addConstraint(autoVoltageConstraint)
            .setReversed(true);
    }

    public static Trajectory loadAutoTrajectory(String name) {
        String fullPath = "output/" + name + ".wpilib.json";
        Trajectory result = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(fullPath);
            
            result = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            result = result.relativeTo(offset);
         } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + fullPath, ex.getStackTrace());
         }

         return result;
    }
}
