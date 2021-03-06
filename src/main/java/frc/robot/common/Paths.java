/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.common;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.subsystems.Drivetrain;
/**
 * Holds autonomous trajectories and methods.
 */
public class Paths {

    private static NetworkTable liveTable = NetworkTableInstance.getDefault().getTable("Live_Dashboard");

    //----- Paths
    public final OCPath forward; // our different paths
    public final OCPath example;
    //-----

    /**
     * Generates autonomous paths given a drivetrain.
     */
    public Paths(Drivetrain drive){
        forward = new OCPath(Poses.forward, drive);
        example = new OCPath(Poses.example, drive);
    }

    /**
     * Lists pose lists for different paths.
     * We explicitly hard-code these waypoints because it is easier to modify/tune quickly--
     * if you would like a visual representation, use FalconDashboard.
     * <b>! Note ! - Pose values are in feet, but the resulting list is converted to meters.</b>
     */
    public static class Poses{
        // All pose distance measurements are in feet!
        public static final List<Pose2d> forward = feetToMeters(
            new Pose2d(),
            new Pose2d(3, 0, new Rotation2d())  
        );
        public static final List<Pose2d> example = feetToMeters(
            new Pose2d(),
            new Pose2d(4, 2, new Rotation2d()),
            new Pose2d(8, 0, new Rotation2d())
        );
        
        /**
         * Takes pose waypoints in feet, returning the list as poses in meters.
         */
        public static List<Pose2d> feetToMeters(Pose2d... poses){
            return Arrays.asList(poses).stream()
                .map(pose -> new Pose2d(new Translation2d(
                    Units.feetToMeters(pose.getTranslation().getX()),
                    Units.feetToMeters(pose.getTranslation().getY())),
                    pose.getRotation()))
                .collect(Collectors.toList());
        }
        /**
         * Takes pose waypoints in meters, returning the list as poses in feet
         * (Should only be used for telemetry).
         */
        public static List<Pose2d> metersToFeet(Pose2d... poses){
            return Arrays.asList(poses).stream()
                .map(pose -> new Pose2d(new Translation2d(
                    Units.metersToFeet(pose.getTranslation().getX()),
                    Units.metersToFeet(pose.getTranslation().getY())),
                    pose.getRotation()))
                .collect(Collectors.toList());
        }
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
}
