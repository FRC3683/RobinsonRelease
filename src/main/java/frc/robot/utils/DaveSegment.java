// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;

/** Add your docs here. */
public class DaveSegment {
    private Trajectory traj;
    private double delay;
    private boolean isWait;
    private double timeOffset;
    private Drivetrain drivetrain;

    public static Pose2d defaultPose = new Pose2d(0, 0, new Rotation2d(0.0));

    public DaveSegment(String s, double timeOffset){
        traj = TrajectoryLoader.loadAutoTrajectory(s);
        isWait = false;
        this.timeOffset = timeOffset;
        drivetrain = Drivetrain.GetInstance();
    }

    public DaveSegment(double delay, double timeOffset){
        this.delay = delay;
        isWait = true;
        this.timeOffset = timeOffset;
        drivetrain = Drivetrain.GetInstance();

    }

    public boolean isDelay(){
        return isWait;
    }

    public Pose2d getInitialPose(){
        if(isWait) return defaultPose;
        else return traj.getInitialPose();
    }

    public Command getCommand(){
        if(isWait) return new WaitCommand(delay);
        else return drivetrain.generateRamseteCommand(traj);
    }

    public double getTotalTime(){
        if(isWait) return delay + timeOffset;
        else return traj.getTotalTimeSeconds() + timeOffset;
    }
}
