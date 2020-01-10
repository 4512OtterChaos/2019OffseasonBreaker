/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import java.util.Arrays;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.ConfigMotors;

/**
 * Subsystem controlling the drivetrain
 */
public class Drivetrain extends SubsystemBase {

    // Enumerator for gearbox states
    public enum Gear {

        LOW(kGearRatioLow), HIGH(kGearRatioHigh);

        private double ratio;

        Gear(double ratio){
            this.ratio = ratio;
        }
        
        public double getRatio(){
            return ratio;
        }
    }
        
    private CANSparkMax leftMotorA = new CANSparkMax(4, MotorType.kBrushless), 
    leftMotorB = new CANSparkMax(5, MotorType.kBrushless),
    leftMotorC = new CANSparkMax(6, MotorType.kBrushless);
    private CANSparkMax rightMotorA = new CANSparkMax(1, MotorType.kBrushless),
    rightMotorB = new CANSparkMax(2, MotorType.kBrushless),
    rightMotorC = new CANSparkMax(3, MotorType.kBrushless);
    
    private CANEncoder leftEncoder;
    private CANEncoder rightEncoder;

    private double driveSpeed = 0.5;
    
    private final CANSparkMax[] motors = {leftMotorA, leftMotorB, leftMotorC, rightMotorA, rightMotorB, rightMotorC};
    
    private final PigeonIMU pigeon = new PigeonIMU(1);
    private double[] ypr = new double[3]; // yaw, pitch, roll degrees
    private double[] xyz = new double[3]; // x, y, z degrees per second
    
    private DoubleSolenoid shifter = new DoubleSolenoid(kShiftForward, kShiftReverse);
    
    private SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(kStaticFF, kVelocityFF, kAccelerationFF);
    private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(kTrackWidthMeters);
    private DifferentialDriveOdometry odometry;
    
    private Gear gear = getShiftedHigh() ? Gear.HIGH : Gear.LOW; // current gearbox ratio
    
    private PIDController leftPIDController = new PIDController(kP, kI, kD, kRobotDelta); // Velocity PID controllers
    private PIDController rightPIDController = new PIDController(kP, kI, kD, kRobotDelta);
    
    public Drivetrain(){
        super();
        
        leftEncoder = leftMotorA.getEncoder();
        rightEncoder = rightMotorA.getEncoder();

        resetEncoders();
        resetGyro();
        
        ConfigMotors.configDriveMotors(true, motors); // set motor configuration

        odometry = new DifferentialDriveOdometry(getHeading());
    }
    
    /**
     * As a subsystem, the periodic function is called by the CommandScheduler to update pertinent values.
     */
    @Override
    public void periodic(){
        updateOdometry();
    }

    //----- Drivetrain control

    public void setDriveSpeed(double speed){
        driveSpeed = speed;
    }
    
    public void setIdleMode(IdleMode mode){
        ConfigMotors.setIdleMode(mode, motors);
    }
    
    /**
     * As opposed to the more basic set() method, using voltage allows for compensation between battery differences.
     * <p>(This method does not actually perform compensation, just turns volts to percentage, and is not affected by drivespeed.
     * See {@link ConfigMotors})
     * @return double[] outputs (0 left, 1 right)
     */
    public void tankDriveVolts(double leftVolts, double rightVolts){
        leftMotorA.setVoltage(leftVolts);
        rightMotorA.setVoltage(rightVolts);
        //return tankDrive(leftVolts / 12.0, rightVolts / 12.0, 1.0);
    }
    /**
     * Sets both sides of the drivetrain to given percentages
     * @return double[] outputs (0 left, 1 right)
     */
    public double[] tankDrive(double left, double right){
        return tankDrive(left, right, driveSpeed);
    }
    /**
     * Sets both sides of the drivetrain to given percentages
     * @param driveSpeed Overloads current drivetrain speed
     * @return double[] outputs (0 left, 1 right)
     */
    public double[] tankDrive(double left, double right, double driveSpeed){
        left *= driveSpeed;
        right *= driveSpeed;
        leftMotorA.set(left);
        rightMotorA.set(right);
        return new double[]{left, right};
    }
    /**
     * Converts forward and turn percentages to tank drive percentages.
     * @return double[] outputs (0 left, 1 right)
     */
    public double[] arcadeDrive(double forward, double turn){
        return arcadeDrive(forward, turn, driveSpeed);
    }
    /**
     * Converts forward and turn percentages to tank drive percentages.
     * @param driveSpeed Overloads current drivetrain speed
     * @return double[] outputs (0 left, 1 right)
     */
    public double[] arcadeDrive(double forward, double turn, double driveSpeed){
        double left = (forward + turn);
        double right = (forward - turn);
        return tankDrive(left, right, driveSpeed);
    }
    /**
     * Uses PID + FF to achieve given wheel speeds.
     * <p><b>! Note: All inputs are in meters and all outputs are in volts.
     */
    public void setVelocityPID(double leftMetersPerSecond, double rightMetersPerSecond){
        DifferentialDriveWheelSpeeds speeds  = getWheelSpeeds();
        //double leftVolts = feedForward.calculate(leftMetersPerSecond);
        //double rightVolts = feedForward.calculate(rightMetersPerSecond);
        tankDriveVolts(
            feedForward.calculate(leftMetersPerSecond)+
            leftPIDController.calculate(speeds.leftMetersPerSecond, leftMetersPerSecond),
            feedForward.calculate(rightMetersPerSecond)+
            rightPIDController.calculate(speeds.rightMetersPerSecond, rightMetersPerSecond));
    }
    public PIDController getLeftPIDController(){
        return leftPIDController;
    }
    public PIDController getRightPIDController(){
        return rightPIDController;
    }
    
    public void resetEncoders(){
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }
    public void resetGyro(){
        pigeon.setYaw(0);
    }
    
    //----- Shifting

    public boolean getShiftedHigh(){
        return (shifter.get() == Value.kForward);
    }
    private void setShifterHigh(boolean is){
        shifter.set(is ? Value.kForward:Value.kReverse);
    }
    
    public void shift(Gear gear){
        this.gear = gear;
        
        if(getShiftedHigh()){
            if(gear == Gear.LOW){
                setShifterHigh(false);
            }
        }
        else{
            if(gear == Gear.HIGH){
                setShifterHigh(true);
            }
        }
    }
    public Gear getReduction(){
        return gear;
    }
    public double getReductionRatio(){
        return gear.getRatio();
    }
    public double getMaxVelocityMeters(){
        return getReduction() == Gear.HIGH ? kMaxMetersHighGear:kMaxMetersLowGear;
    }
    
    //----- 'Meta' methods

    /**
     * @return DifferentialDriveWheelSpeeds object in meters per second
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds(){
        double circumference = Math.PI * 2.0 * kWheelRadiusMeters;
        return new DifferentialDriveWheelSpeeds(
            (leftEncoder.getVelocity() / getReductionRatio() * circumference) / 60.0,
            (rightEncoder.getVelocity() / getReductionRatio() * circumference) / 60.0);
    }
    public double getEncoderDistance(CANEncoder encoder){
        return encoder.getPosition() * Math.PI * 2.0 * kWheelRadiusMeters;
    }

    public SimpleMotorFeedforward getFeedForward(){
        return feedForward;
    }
    
    public DifferentialDriveKinematics getKinematics(){
        return kinematics;
    }
    
    public Rotation2d getHeading(){
        pigeon.getYawPitchRoll(ypr);
        pigeon.getRawGyro(xyz);
        return Rotation2d.fromDegrees(-ypr[0]);
    }
    public DifferentialDriveOdometry getOdometry(){
        return odometry;
    }
    public void updateOdometry(){
        odometry.update(getHeading(), getEncoderDistance(leftEncoder), getEncoderDistance(rightEncoder));
    }
    public void resetOdometry(){
        resetOdometry(getHeading());
    }
    public void resetOdometry(Rotation2d gyroAngle){
        resetOdometry(new Pose2d(), gyroAngle);
    }
    public void resetOdometry(Pose2d poseMeters, Rotation2d gyroAngle){
        resetEncoders();
        odometry.resetPosition(poseMeters, gyroAngle);
    }

    /**
    * Log data to the dashboard.
    */
    public void log(){
        SmartDashboard.putNumber("Gyro Yaw", getHeading().getDegrees());
        SmartDashboard.putString("Gear", gear.toString());
        //SmartDashboard.putNumberArray("Velocities", new double[]{leftEncoder.getVelocity(), rightEncoder.getVelocity()});
        SmartDashboard.putNumber("Left Velocity", leftEncoder.getVelocity());
        SmartDashboard.putNumber("Right Velocity", rightEncoder.getVelocity());
        DifferentialDriveWheelSpeeds speeds = getWheelSpeeds();
        SmartDashboard.putNumber("Left Feet Per", Units.metersToFeet(speeds.leftMetersPerSecond));
        SmartDashboard.putNumber("Right Feet Per", Units.metersToFeet(speeds.rightMetersPerSecond));
        SmartDashboard.putNumber("Left Setpoint", Units.metersToFeet(leftPIDController.getSetpoint()));
        SmartDashboard.putNumber("Right Setpoint", Units.metersToFeet(rightPIDController.getSetpoint()));
        SmartDashboard.putNumber("Left Output", leftMotorA.getAppliedOutput());
        SmartDashboard.putNumber("Right Output", rightMotorA.getAppliedOutput());
        SmartDashboard.putNumber("Linear Feet Per", Units.metersToFeet(kinematics.toChassisSpeeds(getWheelSpeeds()).vxMetersPerSecond));
        SmartDashboard.putNumber("Angular Radians Per", kinematics.toChassisSpeeds(getWheelSpeeds()).omegaRadiansPerSecond);
        NetworkTable liveTable = NetworkTableInstance.getDefault().getTable("Live_Dashboard"); // field positions for live visualizer
        liveTable.getEntry("robotX").setDouble(Units.metersToFeet(getOdometry().getPoseMeters().getTranslation().getX()));
        liveTable.getEntry("robotY").setDouble(Units.metersToFeet(getOdometry().getPoseMeters().getTranslation().getY()));
        liveTable.getEntry("robotHeading").setDouble(getOdometry().getPoseMeters().getRotation().getRadians());

        //SmartDashboard.putData(leftPIDController);
        //SmartDashboard.putData(rightPIDController);
    }
    
}
