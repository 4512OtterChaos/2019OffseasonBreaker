/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.common;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpiutil.math.MathUtil;

/**
 * Custom {@link XboxController} wrapper to add convenience features for driving.
 */
public class OCController extends XboxController{

    private static final double deadband = 0.12;

    /**
     * Constructs OCController on DS joystick port.
     */
    public OCController(int port){
        super(port);
    }

    /**
     * Deadbands a value and re-scales it.
     * @param value Value to adjust
     * @return -1 to 1
     */
    public static double applyDeadband(double value){
        return applyDeadband(value, deadband);
    }
    /**
     * Deadbands a value and re-scales it.
     * @param value Value to adjust
     * @param dead Deadband threshold
     * @return -1 to 1
     */
    public static double applyDeadband(double value, double dead){
        if(Math.abs(value) < dead) return 0;
        double scale = 1.0 / (1 - dead);
        double sign = Math.copySign(1.0, value);
        value = Math.abs(value);
        double fixedValue = sign*(scale*((value-dead)*value));
        return MathUtil.clamp(fixedValue, -1, 1);
    }

    public double getY(Hand hand){
        return -super.getY(hand);
    }
    public double getX(Hand hand){
        return -super.getX(hand);
    }

    /**
     * Applies deadband math to left Y to give 'forward' power.
     * @return Percentage(-1 to 1)
     */
    public double getForward(){
        return applyDeadband(getY(Hand.kLeft));
    }
    /**
     * Applies deadband math to right X to give 'turn' power.
     * @return Percentage(-1 to 1)
     */
    public double getTurn(){
        return applyDeadband(getX(Hand.kRight));
    }

    /**
     * Gives left DT percentage using arcade mode.
     */
    public double getLeftArcade(){
        return getLeftArcade(getForward(), getTurn());
    }
    /**
     * Gives left DT percentage using arcade mode.
     */
    public double getLeftArcade(double forward, double turn){
        return forward - turn;
    }
    /**
     * Gives right DT percentage using arcade mode.
     */
    public double getRightArcade(){
        return getRightArcade(getForward(), getTurn());
    }
    /**
     * Gives right DT percentage using arcade mode.
     */
    public double getRightArcade(double forward, double turn){
        return forward + turn;
    }
}
