/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.common;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.geometry.Pose2d;

/**
 * Common interface that describes basic functionality for a vision module.
 */
public abstract class Vision {
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

    protected NetworkTable visionTable;

    protected State currState = State.DRIVE;

    public void setState(State state){
        currState = state;
        setLedMode(state.getLedMode());
        setPipeline(state.getPipeline());
        setStreamMode(state.getStreamMode());
    }
    protected abstract void setLedMode(int value);
    protected abstract void setPipeline(int value);
    protected abstract void setStreamMode(int value);
    
    public State getState(){
        return currState;
    }
    public abstract double getLedMode();
    public abstract double getPipeline();
    public abstract double getStreamMode();
    public abstract boolean getHasTarget();
    public abstract double getTx();
    public abstract double getTy();
    public abstract double getArea();
    public abstract Pose2d getPose();
}
