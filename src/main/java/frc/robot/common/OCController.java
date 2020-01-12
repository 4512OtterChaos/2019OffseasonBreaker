/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.common;

import edu.wpi.first.wpilibj.XboxController;

/**
 * Add your docs here.
 */
public class OCController extends XboxController{
    public OCController(int port){
        super(port);
    }

    public double applyDeadband(double value){
        return applyDeadband(value, 0.1);
    }
    public double applyDeadband(double value, double dead){
        double scale = 1.0 / (1 - dead);
        double sign = Math.copySign(1.0, value);
        value = Math.abs(value);
        double fixedValue = sign*(scale*((value-dead)*value));
        if(value < 0.1) return 0;
        else return Math.min(fixedValue, 1.0);
    }

    public double getForward(){
        return -applyDeadband(getY(Hand.kRight));
    }
    public double getTurn(){
        return -applyDeadband(getX(Hand.kLeft));
    }

    public double getLeftArcade(){
        return getLeftArcade(getForward(), getTurn());
    }
    public double getLeftArcade(double forward, double turn){
        return forward - turn;
    }
    public double getRightArcade(){
        return getRightArcade(getForward(), getTurn());
    }
    public double getRightArcade(double forward, double turn){
        return forward + turn;
    }
}
