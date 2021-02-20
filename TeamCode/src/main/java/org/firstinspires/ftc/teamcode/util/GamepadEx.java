package org.firstinspires.ftc.teamcode.util;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * A wrapper for the {@link Gamepad} object with toggles and other useful features.
 * <strong>Note:</strong> To work properly, {@link #update()} must be called every loop iteration.
 */
public class GamepadEx {
    //Gamepad Object
    /**
     * Reference to the {@link Gamepad} object this class is using.
     */
    public Gamepad internal;

    //Buttons and Triggers
    public Button a = new Button();
    public Button b = new Button();
    public Button back = new Button();
    public Button circle = new Button();
    public Button cross = new Button();
    public Button dpad_down = new Button();
    public Button dpad_left = new Button();
    public Button dpad_right = new Button();
    public Button dpad_up = new Button();
    public Button guide = new Button();
    public Button left_bumper = new Button();
    public Button left_stick_button = new Button();
    public Trigger left_trigger = new Trigger(0.5f);
    public Button options = new Button();
    public Button ps = new Button();
    public Button right_bumper = new Button();
    public Button right_stick_button = new Button();
    public Trigger right_trigger = new Trigger(0.5f);
    public Button share = new Button();
    public Button square = new Button();
    public Button start = new Button();
    public Button touchpad = new Button();
    public Button triangle = new Button();
    public Button x = new Button();
    public Button y = new Button();

    //Constructor
    public GamepadEx(Gamepad gamepad){
        this.internal = gamepad;
    }

    private Button getButton(GamepadButtons button){
        switch(button){
            case A: return a;
            case B: return b;
            case BACK: return back;
            case X: return x;
            case Y: return y;
            case START: return start;
            case DPAD_UP: return dpad_up;
            case DPAD_DOWN: return dpad_down;
            case DPAD_LEFT: return dpad_left;
            case DPAD_RIGHT: return dpad_right;
            case LEFT_BUMPER: return left_bumper;
            case RIGHT_BUMPER: return right_bumper;
            case LEFT_STICK_BUTTON: return left_stick_button;
            case RIGHT_STICK_BUTTON: return right_stick_button;
            case PS: return ps;
            case CROSS: return cross;
            case GUIDE: return guide;
            case SHARE: return share;
            case CIRCLE: return circle;
            case SQUARE: return square;
            case OPTIONS: return options;
            case TOUCHPAD: return touchpad;
            case TRIANGLE: return triangle;
        }
        return null;
    }
    
    public Trigger getTrigger(GamepadButtons button){
        switch(button){
            case RIGHT_TRIGGER: return right_trigger;
            case LEFT_TRIGGER: return left_trigger;
        }
        return null;
    }
    
    public boolean getState(GamepadButtons button){
        if(button == GamepadButtons.LEFT_TRIGGER || button == GamepadButtons.RIGHT_TRIGGER){
            return getTrigger(button).getState();
        }
        else {
            return getButton(button).getState();
        }
    }

    public boolean setState(GamepadButtons button, boolean state){
        if(button == GamepadButtons.LEFT_TRIGGER || button == GamepadButtons.RIGHT_TRIGGER){
            return false;
        }
        else {
            getButton(button).setState(state);
            return true;
        }
    }

    public boolean justPressed(GamepadButtons button){
        if(button == GamepadButtons.LEFT_TRIGGER || button == GamepadButtons.RIGHT_TRIGGER){
            return getTrigger(button).justPressed();
        }
        else {
            return getButton(button).justPressed();
        }
    }
    
    /**
     * Updates the values of the {@link GamepadEx} object to match those of the actual gamepad ({@link #internal}). Required for the buttons to work.
     */
    public void update(){
        a.setState(internal.a);
        b.setState(internal.b);
        back.setState(internal.back);
        circle.setState(internal.circle);
        cross.setState(internal.cross);
        dpad_down.setState(internal.dpad_down);
        dpad_left.setState(internal.dpad_left);
        dpad_right.setState(internal.dpad_right);
        dpad_up.setState(internal.dpad_up);
        guide.setState(internal.guide);
        left_bumper.setState(internal.left_bumper);
        left_stick_button.setState(internal.left_stick_button);
        left_trigger.setValue(internal.left_trigger);
        options.setState(internal.options);
        ps.setState(internal.ps);
        right_bumper.setState(internal.right_bumper);
        right_stick_button.setState(internal.right_stick_button);
        right_trigger.setValue(internal.right_trigger);
        share.setState(internal.share);
        square.setState(internal.square);
        start.setState(internal.start);
        touchpad.setState(internal.touchpad);
        triangle.setState(internal.triangle);
        x.setState(internal.x);
        y.setState(internal.y);
    }
}