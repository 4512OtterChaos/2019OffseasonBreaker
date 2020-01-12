package frc.robot;

import static frc.robot.common.Constants.*;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.BasicRamseteTest;
import frc.robot.common.OCController;
import frc.robot.common.Trajectories;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {

    private Drivetrain drivetrain;
    private Trajectories trajectories;

    private Compressor compressor = new Compressor();

    private OCController controller = new OCController(0);

    private SendableChooser<Command> commandChooser = new SendableChooser<>();

    public RobotContainer() {
        drivetrain = new Drivetrain();
        trajectories = new Trajectories(drivetrain.getKinematics());

        drivetrain.shift(Drivetrain.Gear.LOW);

        configureButtonBindings(); // Attach functionality to controller

        commandChooser.setDefaultOption("Nothing", new InstantCommand(() -> drivetrain.tankDrive(0,0), drivetrain));
        commandChooser.addOption("Ramsete Test", 
            new BasicRamseteTest(drivetrain, trajectories)
                .andThen(() -> drivetrain.tankDrive(0,0))
        );
        SmartDashboard.putData("Auto mode", commandChooser);
    }

    public void log(){
        drivetrain.log();

        SmartDashboard.putNumber("Left Y", getY(Hand.kLeft));
        SmartDashboard.putNumber("Right X", getX(Hand.kRight));
    }

    private void configureButtonBindings(){
        RunCommand arcadeDrive = new RunCommand(
            ()->{
                double left = controller.getLeftArcade();
                double right = controller.getRightArcade();
                drivetrain.tankDrive(left, right);
            }, 
            drivetrain);
        RunCommand PIDDrive = new RunCommand(
            ()->{
                double linear = controller.getForward()*(drivetrain.getReduction()==Drivetrain.Gear.LOW ? kMaxMetersLowGear:kMaxMetersHighGear);
                double angular = (controller.getTurn()*(kMaxRadiansLowGear));
                ChassisSpeeds chassisSpeeds = new ChassisSpeeds(linear, angular, drivetrain.getHeading().getRadians());
                DifferentialDriveWheelSpeeds wheelSpeeds = drivetrain.getKinematics().toWheelSpeeds(chassisSpeeds);
                double leftMetersPerSecond = wheelSpeeds.leftMetersPerSecond;
                double rightMetersPerSecond = wheelSpeeds.rightMetersPerSecond;
                drivetrain.setVelocityPID(leftMetersPerSecond, rightMetersPerSecond);
            },
            drivetrain);

        drivetrain.setDefaultCommand(arcadeDrive);

        new JoystickButton(controller, 1)
            .whenPressed(() -> drivetrain.shift(Drivetrain.Gear.LOW));
        new JoystickButton(controller, 2)
            .whenPressed(() -> drivetrain.shift(Drivetrain.Gear.HIGH));

        new Trigger(() -> controller.getTriggerAxis(Hand.kRight) > 0.1)
            .toggleWhenActive(PIDDrive);

        new JoystickButton(controller, 6)
            .whenPressed(()->drivetrain.setDriveSpeed(1))
            .whenReleased(()->drivetrain.setDriveSpeed(0.5));
    }
    
    public Command getAutonomousCommand(){
        return commandChooser.getSelected();
    }

    public void resetOdometry(){
        drivetrain.resetOdometry();
    }

    public void setDriveCoast(boolean is){
        if(is){
            drivetrain.setIdleMode(IdleMode.kCoast);
        }
        else{
            drivetrain.setIdleMode(IdleMode.kBrake);
        }
    }
    public void stopDrive(){
        drivetrain.tankDrive(0, 0);
    }

    private double getY(Hand hand){
        double y = controller.getY(hand);
        double deadband = 0.1;
        return -(Math.abs(y) < deadband ? 0:y);
    }
    private double getX(Hand hand){
        double x = controller.getX(hand);
        double deadband = 0.1;
        return -(Math.abs(x) < deadband ? 0:x);
    }
}