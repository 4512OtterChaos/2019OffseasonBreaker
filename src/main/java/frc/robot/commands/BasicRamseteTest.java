/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.common.Trajectories;

/**
 * A basic example of using ramsete controller to follow a short path
 */
public class BasicRamseteTest extends RamseteCommand{

    Trajectory trajectory;
    Timer timer = new Timer();

    /**
     * Construct ramsete command using drivetrain and following test path
     */
    public BasicRamseteTest(Drivetrain drivetrain, Trajectories trajectories){
        super(
            trajectories.ramseteTest,
            () -> drivetrain.getOdometry().getPoseMeters(),
            new RamseteController(),
            drivetrain.getFeedForward(),
            drivetrain.getKinematics(),
            drivetrain::getWheelSpeeds,
            drivetrain.getLeftPIDController(),
            drivetrain.getRightPIDController(),
            drivetrain::tankDriveVolts,
            drivetrain
        );

        trajectory = trajectories.ramseteTest;
    }

    @Override
    public void initialize(){
        super.initialize();

        timer.reset();
        timer.start();
    }

    @Override
    public void execute(){
        super.execute();

        Trajectories.logTrajectory(trajectory, timer.get());
    }

    @Override
    public void end(boolean interrupted){
        super.end(interrupted);

        timer.stop();
    }
}
