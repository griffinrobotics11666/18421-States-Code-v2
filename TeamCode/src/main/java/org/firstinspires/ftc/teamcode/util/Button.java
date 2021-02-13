package org.firstinspires.ftc.teamcode.util;

/**
 * A class that makes it easier to deal with booleans, buttons, and toggles
 */
public class Button {
    private boolean lastState = false;
    private boolean state = false;

    public Button(){}
    public Button(Boolean state){
        this.state = state;
        this.lastState = state;
    }

    public boolean toggle(){
        lastState = state;
        state = !state;
        return state;
    }

    public Boolean setState(Boolean state){
        lastState = this.state;
        this.state = state;
        return state;
    }

    public boolean getState(){
        return state;
    }

    public boolean getLastState() {
        return lastState;
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

    public boolean justChanged(){
        if(state != lastState){
            return true;
        }
        else return false;
    }
}
