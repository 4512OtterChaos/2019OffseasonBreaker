/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.common;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Class for interfacing to robot vision, exposing network table values and calculations.
 */
public class Vision {
    public enum State{
        DRIVE(1, 0, 2),
        BASIC(0, 1, 1),
        PNP(0, 2, 1);

        private int ledMode;
        private int pipeline;
        private int streamMode;

        private State(int ledMode, int pipeline, int streamMode){
            this.ledMode = ledMode;
            this.pipeline = pipeline;
            this.streamMode = streamMode;
        }

        public int getLedMode(){
            return ledMode;
        }
        public int getPipeline(){
            return pipeline;
        }
        public int getStreamMode(){
            return streamMode;
        }
    }
    private NetworkTable visionTable;

    private State currState = State.DRIVE;

    public Vision(){
        this(State.DRIVE);
    }
    public Vision(State state){
        visionTable = NetworkTableInstance.getDefault().getTable("limelight");
        
        setState(state);
    }

    public void setState(State state){
        currState = state;
        setLedMode(state.getLedMode());
        setPipeline(state.getPipeline());
        setStreamMode(state.getStreamMode());
    }
    private void setLedMode(int value){
        if(getLedMode() != value)
            visionTable.getEntry("ledMode").setDouble(value);
    }
    private void setPipeline(int value){
        if(getPipeline() != value)
            visionTable.getEntry("pipeline").setDouble(value);
    }
    private void setStreamMode(int value){
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
        return visionTable.getEntry("tv").getDouble(0) == 1.0;
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
    
}
