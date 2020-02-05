/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.common;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj.geometry.Pose2d;

/**
 * Class for interfacing with a Limelight.
 */
public class Limelight extends Vision{
    LinearFilter txFilter = LinearFilter.movingAverage(4);
    public Limelight(){
        this(State.DRIVE);
    }
    public Limelight(State state){
        visionTable = NetworkTableInstance.getDefault().getTable("limelight");
        
        setState(state);
    }

    public void setState(State state){
        currState = state;
        setLedMode(state.getLedMode());
        setPipeline(state.getPipeline());
        setStreamMode(state.getStreamMode());
    }
    protected void setLedMode(int value){
        if(getLedMode() != value)
            visionTable.getEntry("ledMode").setDouble(value);
    }
    protected void setPipeline(int value){
        if(getPipeline() != value)
            visionTable.getEntry("pipeline").setDouble(value);
    }
    protected void setStreamMode(int value){
        if(getStreamMode() != value)
            visionTable.getEntry("stream").setDouble(value);
    }
    

    public State getState(){
        return currState;
    }
    public double getLedMode(){
        return visionTable.getEntry("ledMode").getDouble(0);
    }
    public double getPipeline(){
        return visionTable.getEntry("pipeline").getDouble(0);
    }
    public double getStreamMode(){
        return visionTable.getEntry("stream").getDouble(0);
    }
    public boolean getHasTarget(){
        return visionTable.getEntry("tv").getDouble(0) != 0;
    }
    public double getTx(){
        return visionTable.getEntry("tx").getDouble(0);
    }
    public double getTy(){
        return visionTable.getEntry("ty").getDouble(0);
    }
    public double getArea(){
        return visionTable.getEntry("area").getDouble(0);
    }
    public Pose2d getPose(){
        return new Pose2d();
    }

    public double getFilteredTx(){
        return txFilter.calculate(getTx());
    }
    
}
