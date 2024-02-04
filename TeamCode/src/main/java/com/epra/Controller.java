package com.epra;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.Gamepad;
//Queer coded - Zachy K
public class Controller extends Gamepad {
    Gamepad gamepad = new Gamepad();
    public static enum Button {
        A (0),
        B (1),
        X (2),
        Y (3),
        UP (4),
        DOWN (5),
        LEFT (6),
        RIGHT (7),
        BUMPER_LEFT (8),
        BUMPER_RIGHT (9),
        STICK_LEFT (10),
        STICK_RIGHT (11),
        LEFT_STICK_X (12),
        RIGHT_STICK_X (13),
        LEFT_STICK_Y (14),
        RIGHT_STICK_Y (15),
        LEFT_TRIGGER (16),
        RIGHT_TRIGGER (17);

        int num;
        private Button (int num) {this.num = num;}
    }
    //array that stores the flag booleans for singlePress for each button
    private boolean[] flag = new boolean[12];
    //array that stores the value of toggle for each button
    private boolean[] toggle = new boolean[12];
    private int[] counter = new int[12];

    private float deadband = 0.0f;

    /**Extends the Gamepad Class.
     * <p></p>
     * Queer Coded by Zachy K. If you use this class or a method from this class in its entirety, please make sure to give credit.
     * <p></p>
     * Introduces new functionality to joysticks, triggers, and buttons.
     * <p></p>
     * Joysticks and Triggers:
     * <p>
     * Joysticks and Triggers return float values. Joysticks return values between -1.0 and 1.0. Triggers return values between 0.0 and 1.0.
     * <p>
     * Deadbanding - A range from the negative value of deadband to the positive value of deadband. If a joystick or trigger's output is withing this range, the output will be set to 0.
     * <p>
     * Pow - Will return the joystick or trigger's output raised to a certain power.
     * <p></p>
     * Buttons:
     * <p>
     * Buttons return boolean values
     * <p>
     * Single Press - Returns a true output only on the first call while a button is pressed.
     * If the method is called again while the button is still pressed, the return will be false.
     * If the method is called while the button is released it will reset.
     * <p>
     * Toggle - A boolean separate from the button that can be changed with or without button input.
     * */
    public Controller (Gamepad g, float deadbandIn) {
        gamepad = g;
        for (int ii = 0; ii < flag.length; ii++) {flag[ii] = false;}
        for (int ii = 0; ii < toggle.length; ii++) {toggle[ii] = false;}
        for (int ii = 0; ii < counter.length; ii++) {counter[ii] = 0;}
        deadband = deadbandIn;
    }
    /**Sets deadband limit for joysticks and triggers.*/
    public void setDeadband(float d) {deadband = d;}
    /**Returns deadband limit for joysticks and triggers.*/
    public float getDeadband() {return deadband;}
    /**Returns the value of the analog control.*/
    public float analogCase (Button analog) {
        float r = 0;
        switch (analog) {
            case LEFT_STICK_X:
                r = gamepad.left_stick_x;
                break;
            case RIGHT_STICK_X:
                r = gamepad.right_stick_x;
                break;
            case LEFT_STICK_Y:
                r = gamepad.left_stick_y;
                break;
            case RIGHT_STICK_Y:
                r = gamepad.right_stick_y;
                break;
            case LEFT_TRIGGER:
                r = gamepad.left_trigger;
                break;
            case RIGHT_TRIGGER:
                r = gamepad.right_trigger;
                break;
        }
        return r;
    }
    /**Returns 0 if in the deadband range, if not returns as normal.*/
    public float analogDeadband (Button analog) {return (Math.abs(analogCase(analog)) > deadband) ? analogCase(analog) : 0.0F;}
    /**Returns 0 if in the deadband range, if not returns as normal.*/
    public float analogDeadband (Button analog, float deadbandIn) {return (Math.abs(analogCase(analog)) > deadbandIn) ? analogCase(analog) : 0.0F;}
    /**Returns the value raised to the power of the input.*/
    public float analogPower(Button analog, int power) {return Math.signum(analogCase(analog) * (float)Math.pow(Math.abs(analogCase(analog)), power));}
    /**If the value is within the deadband range, it is set to 0. If not, it is raised to the power of the input.*/
    public float analogPowerDeadband(Button analog, int power) {return (Math.abs(analogPower(analog, power)) > deadband) ? analogPower(analog, power) : 0.0F;}
    /**If the value is within the deadband range, it is set to 0. If not, it is raised to the power of the input.*/
    public float analogPowerDeadband(Button analog, int power, int deadbandIn) {return (Math.abs(analogPower(analog, power)) > deadbandIn) ? analogPower(analog, power) : 0.0F;}

    /**Returns the state of the button.*/
    public boolean buttonCase (Button button) {
        boolean r = false;
        switch (button) {
            case A:
                r = gamepad.a;
                break;
            case B:
                r = gamepad.b;
                break;
            case X:
                r = gamepad.x;
                break;
            case Y:
                r = gamepad.y;
                break;
            case UP:
                r = gamepad.dpad_up;
                break;
            case DOWN:
                r = gamepad.dpad_down;
                break;
            case LEFT:
                r = gamepad.dpad_left;
                break;
            case RIGHT:
                r = gamepad.dpad_right;
                break;
            case BUMPER_LEFT:
                r = gamepad.left_bumper;
                break;
            case BUMPER_RIGHT:
                r = gamepad.right_bumper;
                break;
            case STICK_LEFT:
                r = gamepad.left_stick_button;
                break;
            case STICK_RIGHT:
                r = gamepad.right_stick_button;
                break;
        }
        return r;
    }
    /**Returns the output of buttonCase as an Integer*/
    public int buttonCaseInt(Button button) {return boolToInt(buttonCase(button));}
    /**Returns a true output only on the first call while a button is pressed.
     * If the method is called again while the button is still pressed, the return will be false.
     * If the method is called while the button is released it will reset.*/
    public boolean buttonSingle(Button button) {
        boolean r = false;
        if (buttonCase(button)) {
            if (!flag[button.num]) {
                r = true;
                flag[button.num] = true;
            }
        } else {
            flag[button.num] = false;
        }
        return r;
    }
    /**Returns the output of buttonSingle as an Integer*/
    public int buttonSingleInt(Button button) {return boolToInt(buttonSingle(button));}
    /**Will change the state of the toggle if the button is pressed.
     * Returns the new state of the toggle.*/
    public boolean buttonToggle(Button button) {
        if (buttonCase(button)) {
            toggle[button.num] = !(toggle[button.num]);
        }
        return toggle[button.num];
    }
    /**Returns the output of buttonToggle as an Integer*/
    public int buttonToggleInt(Button button) {return boolToInt(buttonToggle(button));}
    /**Will change the state of the toggle if the button is pressed following the rules of buttonSingle.
     * Returns the new state of the toggle.*/
    public boolean buttonToggleSingle(Button button) {
        if (buttonSingle(button)) {
            toggle[button.num] = !(toggle[button.num]);
        }
        return toggle[button.num];
    }
    /**Returns the output of buttonToggleSingle as an Integer*/
    public int buttonToggleSingleInt(Button button) {return boolToInt(buttonToggleSingle(button));}
    /**Will change the state of the toggle regardless of the state of the button.
     * Returns the new state of the toggle.*/
    public boolean flipToggle(Button button) {
        toggle[button.num] = !(toggle[button.num]);
        return toggle[button.num];
    }
    /**Returns the state of the toggle without changing the state of the toggle.*/
    public boolean getToggle(Button button) {
        return toggle[button.num];
    }
    /**Returns the output of getToggle as an Integer*/
    public int getToggleInt(Button button) {return boolToInt(buttonToggleSingle(button));}
    /**If the counter is more than or equal to max it will be reset and return zero. If not, the counter will increase by one and return the result.*/
    public int buttonCounter(Button button, int max) {
        if (buttonCase(button)) {
            counter[button.num] = (counter[button.num + 1]) % max;
        }
        return counter[button.num];
    }
    /**Will perform the same action as buttonCounter but follows the rules of buttonSingle.*/
    public int buttonCounterSingle(Button button, int max) {
        if (buttonSingle(button)) {
            counter[button.num] = (counter[button.num] + 1) % max;
        }
        return counter[button.num];
    }
    /**Will increase the counter of a certain button by a certain amount. If the counter goes over max, it will reset and overflow. Returns the new value of the counter.*/
    public int increaseCounter(Button button, int max, int increase) {
        counter[button.num] = (counter[button.num] + increase + max) % max;
        return counter[button.num];
    }
    /**Will set the counter to a certain number.*/
    public void setCounter(Button button, int set) {
        counter[button.num] = set;
    }
    /**Returns the current value of the counter*/
    public int getCounter(Button button) {return counter[button.num];}

    /**If true will return 1, if false will return 0.*/
    public int boolToInt(boolean b) {return (b) ? 1 : 0;}
}