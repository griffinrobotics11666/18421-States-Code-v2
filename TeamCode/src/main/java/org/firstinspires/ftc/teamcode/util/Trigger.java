package org.firstinspires.ftc.teamcode.util;

/**
 * A class that makes it easier to deal with gamepad triggers.
 */
public class Trigger {
    private float value = 0.0f;
    private float lastValue = 0.0f;
    private boolean state = false;
    private boolean lastState = false;
    private float threshold;

    public Trigger(){}
    public Trigger(float threshold){
        this.threshold = threshold;
    }

    public float setThreshold(float threshold){
        this.threshold = threshold;
        return this.threshold;
    }
    public float getThreshold(){
        return this.threshold;
    }

    public float getValue(){
        return value;
    }
    public void setValue(float value){
        lastValue = this.value;
        this.value = value;
        if(this.value>threshold){
            lastState = state;
            state = true;
        }
        else {
            lastState = state;
            state = false;
        }
    }

    public boolean getState(){
        return state;
    }

    public boolean isPressed(){
        return state;
    }

    public boolean justPressed(){
        if(state && !lastState){
            return true;
        }
        else return false;
    }

    public boolean justReleased(){
        if(!state && lastState){
            return true;
        }
        else return false;
    }

    public boolean stateJustChanged(){
        if(state != lastState){
            return true;
        }
        else return false;
    }

    public boolean justMovedFromRest(){
        if(lastValue == 0 && value >0){
            return true;
        }
        else return false;
    }

    public boolean justMoved(){
        if(lastValue != value){
            return true;
        }
        else return false;
    }

    public void update(float value){
        lastValue = this.value;
        this.value = value;
        if(this.value>threshold){
            lastState = state;
            state = true;
        }
        else {
            lastState = state;
            state = false;
        }
    }
}
