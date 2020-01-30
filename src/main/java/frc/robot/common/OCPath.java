/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.common;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

import static frc.robot.common.Constants.*;

import frc.robot.common.Paths.Poses;
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

    public OCPath(List<Pose2d> poses, Drivetrain drive){
        this(TrajectoryGenerator.generateTrajectory(poses, getDefaultConfig(drive)).getStates(), getDefaultConfig(drive));
    }
    public OCPath(List<State> states, TrajectoryConfig config){
        super(states);
        /*
        System.out.println("--States---");
        for(State state:states){
            System.out.println(state.toString());
        }
        System.out.println("----------");
        System.out.println("Config reversed: "+(config.isReversed()));
        */
        this.config = config;
    }
    /*
    public OCPath(List<State> states, TrajectoryConfig config){
        super(states);
    }
    */

    public static TrajectoryConfig getDefaultConfig(Drivetrain drive){
        return new TrajectoryConfig(kMaxMetersLowGear-1, kMaxAccelerationMeters-0.3)
            .setKinematics(drive.getKinematics())
            .addConstraint(new CentripetalAccelerationConstraint(kMaxCentripetalAccelerationMeters)) // Take corners slow
            .addConstraint(new DifferentialDriveVoltageConstraint(drive.getFeedForward(), drive.getKinematics(), 10)); // Account for voltage sag
    }
    /*
    public List<Pose2d> getPoses(){
        List<Pose2d> poses = this.getStates().stream().map(state -> state.poseMeters).collect(Collectors.toList());
        return poses;
    }*/
    public TrajectoryConfig getConfig(){
        return new TrajectoryConfig(config.getMaxVelocity(), config.getMaxAcceleration())
            .addConstraints(config.getConstraints())
            .setReversed(config.isReversed());
    }

    public OCPath getReversed(){
        return new OCPath(getReversedStates(getStates()), 
            getReversedConfig(getConfig()));
    }
    public static List<Pose2d> getReversedPoses(List<Pose2d> poses){
        List<Pose2d> reversedPoses = poses.stream().collect(Collectors.toList());
        Collections.reverse(reversedPoses);
        return reversedPoses;
    }
    public static List<State> getReversedStates(List<State> states){
        List<State> reversedStates = states.stream().collect(Collectors.toList());
        Collections.reverse(reversedStates);
        List<State> reference = reversedStates.stream().collect(Collectors.toList());
        
        for(int i=reversedStates.size();i>0;i--){
            //reversedStates.get(reversedStates.size()-i).timeSeconds = reference.get(i-1).timeSeconds;
            //reversedStates.get(reversedStates.size()-i).velocityMetersPerSecond *= -1;
        }
        return reversedStates;
    }
    public static TrajectoryConfig getReversedConfig(TrajectoryConfig config){
        return new TrajectoryConfig(config.getMaxVelocity(), config.getMaxAcceleration())
            .addConstraints(config.getConstraints())
            .setReversed(!config.isReversed());
    }

}
