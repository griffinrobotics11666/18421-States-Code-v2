package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * A wrapper for both Driver Station gamepads using {@link GamepadEx}.
 */
public class MultipleGamepad {
    public GamepadEx g1;
    public GamepadEx g2;

    public MultipleGamepad(Gamepad gamepad1, Gamepad gamepad2){
        g1 = new GamepadEx(gamepad1);
        g2  = new GamepadEx(gamepad2);
    }

    public void update(){
        g1.update();
        g2.update();
    }
}
