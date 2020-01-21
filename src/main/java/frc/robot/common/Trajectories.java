/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.common;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.common.Constants.*;

/**
 * Holds autonomous trajectories and methods.
 */
public class Trajectories {

    private static NetworkTable liveTable = NetworkTableInstance.getDefault().getTable("Live_Dashboard");

    //----- Trajectories
    public final Trajectory forward; // our different paths
    public final Trajectory backward;
    public final Trajectory example;
    public final Trajectory exampleBackwards;
    //-----

    /**
     * Generates autonomous paths given a drivetrain.
     */
    public Trajectories(Drivetrain drive){
        // Configuration, or behavior, of the path following
        TrajectoryConfig config = new TrajectoryConfig(kMaxMetersLowGear, kMaxAccelerationMeters)
            .setKinematics(drive.getKinematics()) // Measure geometry
            .addConstraint(new CentripetalAccelerationConstraint(kMaxCentripetalAccelerationMeters)) // Take corners slow
            .addConstraint(new DifferentialDriveVoltageConstraint(drive.getFeedForward(), drive.getKinematics(), 10)); // Account for voltage sag

        List<Pose2d> examplePoses = 
            Arrays.asList(
                new Pose2d(),
                new Pose2d(1.2, 0.8, new Rotation2d()),
                new Pose2d(2.4, 0, new Rotation2d())
            );
        List<Pose2d> examplePosesReverse = examplePoses.stream().collect(Collectors.toList());
        Collections.reverse(examplePosesReverse);

        // Straight forward 1 meter
        forward = TrajectoryGenerator.generateTrajectory(
            Arrays.asList(
                new Pose2d(),
                new Pose2d(1, 0, new Rotation2d())
            ), 
            config.setReversed(false));
        
        backward = TrajectoryGenerator.generateTrajectory(
            Arrays.asList(
                new Pose2d(1, 0, new Rotation2d()),
                new Pose2d()
            ), 
            config.setReversed(true));
        // Take a little detour left and back while going forward
        example = TrajectoryGenerator.generateTrajectory(
            examplePoses,
            config.setReversed(false));
        // ...but in reverse
        exampleBackwards = TrajectoryGenerator.generateTrajectory(
            examplePosesReverse,
            config.setReversed(true));

        // For Pathweaver, do TrajectoryUtil.fromPathweaverJson()
    }

    /**
     * Logs an instantaneous point in a trajectory to the live visualizer.
     * @param trajectory Trajectory to track
     * @param timeSeconds Seconds at which pose is sampled (e.g. time since trajectory has started)
     */
    public static void logTrajectory(Trajectory trajectory, double timeSeconds){
        Pose2d currPose = trajectory.sample(timeSeconds).poseMeters; // current pose
        liveTable.getEntry("pathX").setDouble(Units.metersToFeet(currPose.getTranslation().getX())+5);
        liveTable.getEntry("pathY").setDouble(Units.metersToFeet(currPose.getTranslation().getY())+13);
        liveTable.getEntry("isFollowingPath").setBoolean(timeSeconds <= trajectory.getTotalTimeSeconds());
    }

    /**
     * Lists different poses for paths,  as well as utilities for constructing them.
     */
    public static class Paths{
        public Paths(){
        }

        public static List<Pose2d> getPoses(Trajectory traj){
            return traj.getStates().stream().map(state -> state.poseMeters).collect(Collectors.toList());
        }

        public static Trajectory feetToMeters(Trajectory traj, TrajectoryConfig config){
            return TrajectoryGenerator.generateTrajectory(feetToMeters(getPoses(traj)), config);
        }
        public static List<Pose2d> feetToMeters(List<Pose2d> poses){
            return poses.stream()
                .map(pose -> new Pose2d(new Translation2d(
                    Units.feetToMeters(pose.getTranslation().getX()),
                    Units.feetToMeters(pose.getTranslation().getY())),
                    pose.getRotation()))
                .collect(Collectors.toList());
        }

        public static List<Pose2d> reversePoses(Trajectory traj){
            return reversePoses(getPoses(traj));
        }
        public static List<Pose2d> reversePoses(List<Pose2d> poses){
            List<Pose2d> reversedPoses = poses.stream().collect(Collectors.toList());
            Collections.reverse(reversedPoses);
            return reversedPoses;
        }
        
    }
}
