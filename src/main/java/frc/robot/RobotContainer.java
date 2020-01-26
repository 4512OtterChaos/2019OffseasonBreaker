package frc.robot;

import static frc.robot.common.Constants.*;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.BasicRamseteTest;
import frc.robot.common.OCController;
import frc.robot.common.Paths;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {

    private Drivetrain drivetrain;
    private Paths paths;

    private OCController controller = new OCController(0);

    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;

    private SendableChooser<Command> commandChooser = new SendableChooser<>();

    public RobotContainer() {
        drivetrain = new Drivetrain();
        paths = new Paths(drivetrain);

        drivetrain.shift(Drivetrain.Gear.LOW);

        led = new AddressableLED(9);
        ledBuffer = new AddressableLEDBuffer(149);
        led.setLength(ledBuffer.getLength());

        for(int i=0;i<ledBuffer.getLength();i++){
            ledBuffer.setHSV(i, 113, 122, 150);
        }

        led.setData(ledBuffer);
        led.start();

        configureButtonBindings(); // Attach functionality to controller

        commandChooser.setDefaultOption("Nothing", new InstantCommand(() -> drivetrain.tankDrive(0,0), drivetrain));
        commandChooser.addOption("Forward", 
            new BasicRamseteTest(drivetrain, paths.forward)
        );
        commandChooser.addOption("Forward Cycle", 
            new BasicRamseteTest(drivetrain, paths.forward)
            .andThen(new BasicRamseteTest(drivetrain, paths.forward.getReversed()))
        );
        commandChooser.addOption("Test", 
            new BasicRamseteTest(drivetrain, paths.example)
        );
        commandChooser.addOption("Test Cycle",
            new BasicRamseteTest(drivetrain, paths.example)
            .andThen(new BasicRamseteTest(drivetrain, paths.example.getReversed()))
        );
        SmartDashboard.putData("Auto mode", commandChooser);
    }

    public void log(){
        drivetrain.log();
        led.setData(ledBuffer);

        SmartDashboard.putNumber("Left Y", controller.getForward());
        SmartDashboard.putNumber("Right X", controller.getTurn());
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
                ChassisSpeeds chassisSpeeds = new ChassisSpeeds(linear, 0, angular);
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
        drivetrain.resetOdometry(new Rotation2d());
    }

    public void setDriveCoast(boolean is){
        if(is){
            drivetrain.setIdleMode(IdleMode.kCoast);
        }
        else{
            drivetrain.setIdleMode(IdleMode.kBrake);
        }
    }
    public void disable(){
        drivetrain.tankDrive(0, 0);
        setDriveCoast(true);
    }
}