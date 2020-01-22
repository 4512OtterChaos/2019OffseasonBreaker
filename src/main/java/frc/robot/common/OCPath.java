/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.common;

import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

import static frc.robot.common.Constants.*;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;

/**
 * Add your docs here.
 */
public class OCPath {
    public Trajectory trajectory;
    private TrajectoryConfig config;

    private static final TrajectoryConfig defaultConfig = new TrajectoryConfig(kMaxMetersLowGear, kMaxAccelerationMeters)
        .addConstraint(new CentripetalAccelerationConstraint(kMaxCentripetalAccelerationMeters)); // Take corners slow

    public OCPath(List<Pose2d> poses, Drivetrain drive){
        
    }
    public OCPath(List<Pose2d> poses, TrajectoryConfig config){
        this(
            TrajectoryGenerator.generateTrajectory(
                poses, config),
            config
        );
    }
    
    public OCPath(Trajectory traj, TrajectoryConfig config){
        trajectory = traj;
        this.config = config;
    }

    private TrajectoryConfig configDefault(Drivetrain drive){
        defaultConfig.setKinematics(drive.getKinematics()) // Measure geometry
            .addConstraint(new DifferentialDriveVoltageConstraint(drive.getFeedForward(), drive.getKinematics(), 10)); // Account for voltage sag
        return defaultConfig;
    }
  
    public List<Pose2d> getPoses(){
        return trajectory.getStates().stream().map(state -> state.poseMeters).collect(Collectors.toList());
    }

    public Trajectory reversePoses(Trajectory traj, TrajectoryConfig config){
        List<Pose2d> poses = reversePoses(getPoses());
        return TrajectoryGenerator.generateTrajectory(
            poses, config);
    }
    public List<Pose2d> reversePoses(List<Pose2d> poses){
        List<Pose2d> reversedPoses = poses.stream().collect(Collectors.toList());
        Collections.reverse(reversedPoses);
        return reversedPoses;
    }

    //public Trajectory getReversed(){
    //}

}
