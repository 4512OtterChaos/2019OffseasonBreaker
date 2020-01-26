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
 * C
 */
public class OCPath extends Trajectory{
    private TrajectoryConfig config;

    public OCPath(List<Pose2d> poses, Drivetrain drive){
        this(poses, getDefaultConfig(drive));
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
        return this.getStates().stream().map(state -> state.poseMeters).collect(Collectors.toList());
    }

    public OCPath getReversed(){
        List<Pose2d> reversedPoses = getPoses();
        Collections.reverse(reversedPoses);

        OCPath reversedPath = new OCPath(reversedPoses, config.setReversed(true));
        config.setReversed(false);
        return reversedPath;
    }

}
