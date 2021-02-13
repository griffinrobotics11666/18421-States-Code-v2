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

    public boolean getState(GamepadButtons button){
        switch(button){
            case A: return a.getState();
            case B: return b.getState();
            case BACK: return back.getState();
            case X: return x.getState();
            case Y: return y.getState();
            case START: return start.getState();
            case DPAD_UP: return dpad_up.getState();
            case DPAD_DOWN: return dpad_down.getState();
            case DPAD_LEFT: return dpad_left.getState();
            case DPAD_RIGHT: return dpad_right.getState();
            case LEFT_BUMPER: return left_bumper.getState();
            case LEFT_TRIGGER: return left_trigger.getState();
            case RIGHT_BUMPER: return right_bumper.getState();
            case RIGHT_TRIGGER: return right_trigger.getState();
            case LEFT_STICK_BUTTON: return left_stick_button.getState();
            case RIGHT_STICK_BUTTON: return right_stick_button.getState();
            case PS: return ps.getState();
            case CROSS: return cross.getState();
            case GUIDE: return guide.getState();
            case SHARE: return share.getState();
            case CIRCLE: return circle.getState();
            case SQUARE: return square.getState();
            case OPTIONS: return options.getState();
            case TOUCHPAD: return touchpad.getState();
            case TRIANGLE: return triangle.getState();
        }
        return false;
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