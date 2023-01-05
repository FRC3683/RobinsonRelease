// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

/** Add your docs here. */
public class TrajectoryLoader {

    private static Pose2d offset = new Pose2d(0.427, 0.302, Rotation2d.fromDegrees(0));

    private TrajectoryLoader() {
    }

    private static Trajectory createTrajectoryFromElements(double[] elements) {
        // Make sure that the elements have the correct length.

        // Create a list of states from the elements.
        List<Trajectory.State> states = new ArrayList<Trajectory.State>();
        for (int i = 0; i < elements.length; i += 7) {
            states.add(
                    new Trajectory.State(
                            elements[i],
                            elements[i + 1],
                            elements[i + 2],
                            new Pose2d(elements[i + 3], elements[i + 4], new Rotation2d(elements[i + 5])),
                            elements[i + 6]));
        }
        return new Trajectory(states);
    }

    public static Trajectory loadAutoTrajectory(String name) {
        String fullPath = "output/" + name + ".wpilib.json";
        Trajectory result = new Trajectory();
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(fullPath);
        try {
            if (new File(trajectoryPath.toString()).exists()) {
                System.out.println("AAAAAAAAAAAAAAAAAAAAAAAAAAa");
            } else
                throw new IOException();
            
            //result = createTrajectoryFromElements(elements);
            result = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            //result = result.relativeTo(offset);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryPath.toString() + " " + ex.getMessage(), ex.getStackTrace());
            result = new Trajectory();
        }

        return result;
    }
}
