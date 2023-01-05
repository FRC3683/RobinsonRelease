// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

/** Add your docs here. */
public class SparkRamseteCommand extends CommandBase {
    public interface QuadConsumer<T,U,V,W> {
        void accept(T t, U u, V v, W w);
    }

    private final Timer m_timer = new Timer();
    private final Trajectory m_trajectory;
    private final Supplier<Pose2d> m_pose;
    private final RamseteController m_follower;
    private final DifferentialDriveKinematics m_kinematics;
    private final QuadConsumer<Double, Double, Double, Double> m_output;
    private DifferentialDriveWheelSpeeds m_prevSpeeds;
    private double m_prevTime;


    /**
     * Constructs a new RamseteCommand that, when executed, will follow the provided trajectory.
     * Performs no PID control and calculates no feedforwards; outputs are the raw wheel speeds from
     * the RAMSETE controller, and will need to be converted into a usable form by the user.
     *
     * @param trajectory The trajectory to follow.
     * @param pose A function that supplies the robot pose - use one of the odometry classes to
     *     provide this.
     * @param follower The RAMSETE follower used to follow the trajectory.
     * @param kinematics The kinematics for the robot drivetrain.
     * @param outputMetersPerSecond A function that consumes the computed left and right wheel speeds.
     * @param requirements The subsystems to require.
     */
    public SparkRamseteCommand(
        Trajectory trajectory,
        Supplier<Pose2d> pose,
        RamseteController follower,
        DifferentialDriveKinematics kinematics,
        QuadConsumer<Double, Double, Double, Double> outputMetersPerSecond,
        Subsystem... requirements) {
      m_trajectory = trajectory;
      m_pose = pose;
      m_follower = follower;
      m_kinematics = kinematics;
      m_output = outputMetersPerSecond;
  
      addRequirements(requirements);
    }

    @Override
    public void initialize() {
        m_prevTime = -1;
        var initialState = m_trajectory.sample(0);
        m_prevSpeeds = m_kinematics.toWheelSpeeds(
                new ChassisSpeeds(
                        initialState.velocityMetersPerSecond,
                        0,
                        initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond));
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        double curTime = m_timer.get();

        if (m_prevTime < 0) {
            m_output.accept(0.0, 0.0, 0.0, 0.0);
            m_prevTime = curTime;
            return;
        }

        var targetWheelSpeeds = m_kinematics.toWheelSpeeds(
                m_follower.calculate(m_pose.get(), m_trajectory.sample(curTime)));

        m_output.accept(targetWheelSpeeds.leftMetersPerSecond, targetWheelSpeeds.rightMetersPerSecond, m_prevSpeeds.leftMetersPerSecond, m_prevSpeeds.rightMetersPerSecond);
        m_prevSpeeds = targetWheelSpeeds;
        m_prevTime = curTime;
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();

        if (interrupted) {
            m_output.accept(0.0, 0.0, 0.0, 0.0);
        }
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
    }

}
