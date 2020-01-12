/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.common;

import static frc.robot.common.Constants.*;

import java.util.Arrays;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

/**
 * Used for configuring motor settings.
 */
public class ConfigMotors {

    
    /**
     * Configures each motor given with drive settings.
     * <p>If using this overload, one half of the array will be inverted(one side), as well as the motors
     * after the first left and right motors becoming followers.
     * <p><b>! Note: Motor count must be even
     * @param rightInverted Is right or left inverted
     * @param motors Array of drive motors, ordered from left to right
     */
    public static void configDriveMotors(boolean rightInverted, CANSparkMax... motors){
        
        configDriveMotors(motors);
        
        int firstRightIndex = motors.length/2;
        if(motors.length > 1 && motors.length%2 == 0){
            for(int i=0;i<motors.length;i++){
                if(i<firstRightIndex){
                    motors[i].setInverted(!rightInverted);
                    if(i>0) motors[i].follow(motors[0]);
                }
                else{
                    motors[i].setInverted(rightInverted);
                    if(i>firstRightIndex) motors[i].follow(motors[firstRightIndex]);
                }
            }
        }

        //if(motors[4].getInverted()==false)  configDriveMotors(rightInverted, motors);
    }
    /**
     * Configures each motor given with drive settings.
     * <p>Does not invert nor set followers.
     * @param motors Array of drive motors
     */
    public static void configDriveMotors(CANSparkMax... motors){
        Arrays.stream(motors).forEach(motor ->{
            // Make sure motor config is clean
            motor.setCANTimeout(50);
            motor.restoreFactoryDefaults();
            
            // Current limits (don't kill the motors)
            motor.setSmartCurrentLimit(40);
            //motor.setSecondaryCurrentLimit(50);
            
            // Ramp motors
            motor.setOpenLoopRampRate(kRampRaw);
            motor.setClosedLoopRampRate(kRampRaw);

            // Tries to compensate output to minimalize differences between battery voltages actively
            //motor.enableVoltageCompensation(12);
            
            // Save config
            motor.burnFlash();
        });
    }

    /**
     * Sets idle mode of given motors.
     * @param mode IdleMode (Brake or Coast)
     * @param motors Motors to set
     */
    public static void setIdleMode(IdleMode mode, CANSparkMax... motors){
        Arrays.stream(motors).forEach(motor -> motor.setIdleMode(mode));
    }

}
