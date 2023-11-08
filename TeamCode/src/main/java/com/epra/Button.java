package com.epra;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Button {
    private boolean pressed = false;
    private boolean flag = true;
    private boolean toggle = false;

    public Button () {}

    public void setPressed(boolean in) {
        pressed = in;
    }

    public boolean getPressed() {
        return pressed;
    }

    public boolean getSingle() {
        boolean r = false;
        if (pressed) {
            if (flag) {
                flag = false;
                r = true;
            }
        } else {
            flag = true;
        }
        return r;
    }
    //if in is true swaps toggle, then returns the value of toggle
    public boolean getToggle(boolean inVal) {
        if (inVal) {
            toggle = (toggle) ? false : true;
        }
        return toggle;
    }

    public boolean getToggle() {
        if (getSingle()) {
            toggle = (toggle) ? false : true;
        }
        return toggle;
    }
}
