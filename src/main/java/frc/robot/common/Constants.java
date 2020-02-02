/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.common;

import edu.wpi.first.wpilibj.util.Units;

public class Constants {

    public static final double kRobotDelta = 0.01;
    
    // All distance measurements should be in meters when being used
    
    //--------------------Drivetrain
    public static final double kTrackWidthMeters = Units.inchesToMeters(23); // Basically horizontal wheel gap
    public static final double kWheelRadiusMeters = Units.inchesToMeters(2);
    
    public static final int kNeoTicks = 42; // internal neo encoder
    public static final double kGearRatioLow = 11.12; // shifting gearbox ratio(motor rotations per wheel rotation)
    public static final double kGearRatioHigh = 4.41;

    public static final int kDriveStallCurrentLimit = 65;
    public static final int kDriveFreeCurrentLimit = 35;

    public static final double kRampRaw = 0.08; // seconds to full output(on the motor)
    
    public static final double kStaticFF = 0.183; // volts (Given from the characterization tool)
    public static final double kVelocityFF = 4.29; // meters per second
    public static final double kAccelerationFF = 0.428; // meters per second squared
    
    public static final double kP = 1; // PID Gains (For one meter/second of error, kP volts are applied)
    public static final double kI = 0; // These shouldn't be necessary for velocity
    public static final double kD = 0;
    
    public static final int kShiftForward = 0; // PCM channels used for controlling ball shifters
    public static final int kShiftReverse = 1;
    //--------------------
    
    //--------------------Autonomous
    public static final double kMaxMetersLowGear = Units.feetToMeters(8); // Velocity (8 actual)
    public static final double kMaxMetersHighGear = Units.feetToMeters(15);
    public static final double kMaxAccelerationMeters = Units.feetToMeters(4);
    public static final double kMaxRadiansLowGear = Units.degreesToRadians(450); // Rotations per second
    public static final double kMaxCentripetalAccelerationMeters = Units.feetToMeters(3.4); // Turning acceleration given radius
    //--------------------
}
