/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.common;

import static frc.robot.common.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

/**
 * Used for configuring motor settings.
 */
public class ConfigMotors {

    /**
     * Configures each motor given with drive settings.
     * Sets followers and inverts.
     * @param isRightInverted Is right or left inverted
     */
    public static void configDriveMotors(CANSparkMax[] leftMotors, CANSparkMax[] rightMotors, boolean isRightInverted){
        for(int i=0;i<leftMotors.length;i++){
            leftMotors[i].setInverted(!isRightInverted);
            if(i>0) leftMotors[i].follow(leftMotors[0]);
        }
        for(int i=0;i<rightMotors.length;i++){
            rightMotors[i].setInverted(isRightInverted);
            if(i>0) rightMotors[i].follow(rightMotors[0]);
        }

        configDriveMotors(leftMotors);
        configDriveMotors(rightMotors);
    }
    /**
     * Configures each motor given with drive settings.
     * <p>Does not invert nor set followers.
     * @param motors Array of drive motors
     */
    public static void configDriveMotors(CANSparkMax... motors){
        for(CANSparkMax motor:motors){
            // Make sure motor config is clean
            motor.restoreFactoryDefaults();

            configMotors(motors);
            
            // Save config
            motor.burnFlash();
        };
    }
    public static void configMotors(CANSparkMax... motors){
        configMotors(kDriveStallCurrentLimit, kDriveFreeCurrentLimit, motors);
    }
    public static void configMotors(int stallLimit, int freeLimit, CANSparkMax... motors){
        for(CANSparkMax motor:motors){
            // Ramp motors
            motor.setOpenLoopRampRate(kRampRaw);
            motor.setClosedLoopRampRate(kRampRaw);

            // Current limits (don't kill the motors)
            motor.setSmartCurrentLimit(stallLimit, freeLimit);
        }
    }

    /**
     * Sets idle mode of given motors.
     * @param mode IdleMode (Brake or Coast)
     * @param motors Motors to set
     */
    public static void setIdleMode(IdleMode mode, CANSparkMax... motors){
        for(CANSparkMax motor:motors) motor.setIdleMode(mode);
    }

}
