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
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;

/**
 * 
 */
public class OCPath extends Trajectory{
    private TrajectoryConfig config;
    private Drivetrain drive;

    public OCPath(List<Pose2d> poses, Drivetrain drive){
        this(poses, getDefaultConfig(drive));
        this.drive = drive;
    }
    public OCPath(List<Pose2d> poses, TrajectoryConfig config){
        super(TrajectoryGenerator.generateTrajectory(poses, config).getStates());
        this.config = config;
    }

    private static TrajectoryConfig getDefaultConfig(Drivetrain drive){
        return new TrajectoryConfig(kMaxMetersLowGear, kMaxAccelerationMeters)
            .setKinematics(drive.getKinematics())
            .addConstraint(new CentripetalAccelerationConstraint(kMaxCentripetalAccelerationMeters)) // Take corners slow
            .addConstraint(new DifferentialDriveVoltageConstraint(drive.getFeedForward(), drive.getKinematics(), 10)); // Account for voltage sag
    }
  
    public List<Pose2d> getPoses(){
        List<Pose2d> poses = this.getStates().stream().map(state -> state.poseMeters).collect(Collectors.toList());
        System.out.println("--Normal--");
        for(Pose2d pose:poses){
            System.out.println(pose.toString());
        }
        System.out.println("----------");
        return poses;
    }

    public OCPath getReversed(){
        List<Pose2d> reversedPoses = getPoses();
        Collections.reverse(reversedPoses);
        
        System.out.println("-Reversed-");
        for(Pose2d pose:reversedPoses){
            System.out.println(pose.toString());
        }
        System.out.println("----------");

        OCPath reversedPath = new OCPath(reversedPoses, getDefaultConfig(drive).setReversed(true));
        return reversedPath;
    }

}
