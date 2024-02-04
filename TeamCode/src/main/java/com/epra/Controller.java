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
        STICK_RIGHT (11);

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

    public float left_stick_x() {return gamepad.left_stick_x;}
    public float left_stick_y() {return gamepad.left_stick_y;}
    public float right_stick_x() {return gamepad.right_stick_x;}
    public float right_stick_y() {return gamepad.right_stick_y;}
    public float left_trigger() {return gamepad.left_trigger;}
    public float right_trigger() {return gamepad.right_trigger;}

    /**Returns 0 if in the deadband range, if not returns as normal.*/
    public float left_stick_x_deadband() {return (gamepad.left_stick_x > deadband || gamepad.left_stick_x < -1.0 * deadband) ? gamepad.left_stick_x : 0.0F;}
    /**Returns 0 if in the deadband range, if not returns as normal.*/
    public float left_stick_y_deadband() {return (gamepad.left_stick_y > deadband || gamepad.left_stick_y < -1.0 * deadband) ? gamepad.left_stick_y : 0.0F;}
    /**Returns 0 if in the deadband range, if not returns as normal.*/
    public float right_stick_x_deadband() {return (gamepad.right_stick_x > deadband || gamepad.right_stick_x < -1.0 * deadband) ? gamepad.right_stick_x : 0.0F;}
    /**Returns 0 if in the deadband range, if not returns as normal.*/
    public float right_stick_y_deadband() {return (gamepad.right_stick_y > deadband || gamepad.right_stick_y < -1.0 * deadband) ? gamepad.right_stick_y : 0.0F;}
    /**Returns 0 if in the deadband range, if not returns as normal.*/
    public float left_trigger_deadband() {return (gamepad.left_trigger > deadband || gamepad.left_trigger < -1.0 * deadband) ? gamepad.left_trigger : 0.0F;}
    /**Returns 0 if in the deadband range, if not returns as normal.*/
    public float right_trigger_deadband() {return (gamepad.right_trigger > deadband || gamepad.right_trigger < -1.0 * deadband) ? gamepad.right_trigger : 0.0F;}

    /**Returns 0 if in the deadband range, if not returns as normal.*/
    public float left_stick_x_deadband(int deadbandIn) {return (gamepad.left_stick_x > deadbandIn || gamepad.left_stick_x < -1.0 * deadbandIn) ? gamepad.left_stick_x : 0.0F;}
    /**Returns 0 if in the deadband range, if not returns as normal.*/
    public float left_stick_y_deadband(int deadbandIn) {return (gamepad.left_stick_y > deadbandIn || gamepad.left_stick_y < -1.0 * deadbandIn) ? gamepad.left_stick_y : 0.0F;}
    /**Returns 0 if in the deadband range, if not returns as normal.*/
    public float right_stick_x_deadband(int deadbandIn) {return (gamepad.right_stick_x > deadbandIn || gamepad.right_stick_x < -1.0 * deadbandIn) ? gamepad.right_stick_x : 0.0F;}
    /**Returns 0 if in the deadband range, if not returns as normal.*/
    public float right_stick_y_deadband(int deadbandIn) {return (gamepad.right_stick_y > deadbandIn || gamepad.right_stick_y < -1.0 * deadbandIn) ? gamepad.right_stick_y : 0.0F;}
    /**Returns 0 if in the deadband range, if not returns as normal.*/
    public float left_trigger_deadband(int deadbandIn) {return (gamepad.left_trigger > deadbandIn || gamepad.left_trigger < -1.0 * deadbandIn) ? gamepad.left_trigger : 0.0F;}
    /**Returns 0 if in the deadband range, if not returns as normal.*/
    public float right_trigger_deadband(int deadbandIn) {return (gamepad.right_trigger > deadbandIn || gamepad.right_trigger < -1.0 * deadbandIn) ? gamepad.right_trigger : 0.0F;}

    /**Returns the value raised to the power of the input.*/
    public float left_stick_x_pow(int power) {return (float)(((power % 2 == 0) ? Math.signum(left_stick_x()) : 1.0f) * Math.pow(left_stick_x(), power));}
    /**Returns the value raised to the power of the input.*/
    public float left_stick_y_pow(int power) {return (float)(((power % 2 == 0) ? Math.signum(left_stick_y()) : 1.0f) * Math.pow(left_stick_y(), power));}
    /**Returns the value raised to the power of the input.*/
    public float right_stick_x_pow(int power) {return (float)(((power % 2 == 0) ? Math.signum(right_stick_x()) : 1.0f) * Math.pow(right_stick_x(), power));}
    /**Returns the value raised to the power of the input.*/
    public float right_stick_y_pow(int power) {return (float)(((power % 2 == 0) ? Math.signum(right_stick_y()) : 1.0f) * Math.pow(right_stick_y(), power));}
    /**Returns the value raised to the power of the input.*/
    public float left_trigger_pow(int power) {return (float)(((power % 2 == 0) ? Math.signum(left_trigger()) : 1.0f) * Math.pow(left_trigger(), power));}
    /**Returns the value raised to the power of the input.*/
    public float right_trigger_pow(int power) {return (float)(((power % 2 == 0) ? Math.signum(right_trigger()) : 1.0f) * Math.pow(right_trigger(), power));}

    /**If the value is within the deadband range, it is set to 0. If not, it is raised to the power of the input.*/
    public float left_stick_x_deadband_pow(int power) {return (float)(((power % 2 == 0) ? Math.signum(left_stick_x_deadband()) : 1.0f) * Math.pow(left_stick_x_deadband(), power));}
    /**If the value is within the deadband range, it is set to 0. If not, it is raised to the power of the input.*/
    public float left_stick_y_deadband_pow(int power) {return (float)(((power % 2 == 0) ? Math.signum(left_stick_y_deadband()) : 1.0f) * Math.pow(left_stick_y_deadband(), power));}
    /**If the value is within the deadband range, it is set to 0. If not, it is raised to the power of the input.*/
    public float right_stick_x_deadband_pow(int power) {return (float)(((power % 2 == 0) ? Math.signum(right_stick_x_deadband()) : 1.0f) * Math.pow(right_stick_x_deadband(), power));}
    /**If the value is within the deadband range, it is set to 0. If not, it is raised to the power of the input.*/
    public float right_stick_y_deadband_pow(int power) {return (float)(((power % 2 == 0) ? Math.signum(right_stick_y_deadband()) : 1.0f) * Math.pow(right_stick_y_deadband(), power));}
    /**If the value is within the deadband range, it is set to 0. If not, it is raised to the power of the input.*/
    public float left_trigger_deadband_pow(int power) {return (float)(((power % 2 == 0) ? Math.signum(left_trigger_deadband()) : 1.0f) * Math.pow(left_trigger_deadband(), power));}
    /**If the value is within the deadband range, it is set to 0. If not, it is raised to the power of the input.*/
    public float right_trigger_deadband_pow(int power) {return (float)(((power % 2 == 0) ? Math.signum(right_trigger_deadband()) : 1.0f) * Math.pow(right_trigger_deadband(), power));}

    /**If the value is within the deadband range, it is set to 0. If not, it is raised to the power of the input.*/
    public float left_stick_x_deadband_pow(int power, int deadbandIn) {return (float)(((power % 2 == 0) ? Math.signum(left_stick_x_deadband(deadbandIn)) : 1.0f) * Math.pow(left_stick_x_deadband(deadbandIn), power));}
    /**If the value is within the deadband range, it is set to 0. If not, it is raised to the power of the input.*/
    public float left_stick_y_deadband_pow(int power, int deadbandIn) {return (float)(((power % 2 == 0) ? Math.signum(left_stick_y_deadband(deadbandIn)) : 1.0f) * Math.pow(left_stick_y_deadband(deadbandIn), power));}
    /**If the value is within the deadband range, it is set to 0. If not, it is raised to the power of the input.*/
    public float right_stick_x_deadband_pow(int power, int deadbandIn) {return (float)(((power % 2 == 0) ? Math.signum(right_stick_x_deadband(deadbandIn)) : 1.0f) * Math.pow(right_stick_x_deadband(deadbandIn), power));}
    /**If the value is within the deadband range, it is set to 0. If not, it is raised to the power of the input.*/
    public float right_stick_y_deadband_pow(int power, int deadbandIn) {return (float)(((power % 2 == 0) ? Math.signum(right_stick_y_deadband(deadbandIn)) : 1.0f) * Math.pow(right_stick_y_deadband(deadbandIn), power));}
    /**If the value is within the deadband range, it is set to 0. If not, it is raised to the power of the input.*/
    public float left_trigger_deadband_pow(int power, int deadbandIn) {return (float)(((power % 2 == 0) ? Math.signum(left_trigger_deadband(deadbandIn)) : 1.0f) * Math.pow(left_trigger_deadband(deadbandIn), power));}
    /**If the value is within the deadband range, it is set to 0. If not, it is raised to the power of the input.*/
    public float right_trigger_deadband_pow(int power, int deadbandIn) {return (float)(((power % 2 == 0) ? Math.signum(right_trigger_deadband(deadbandIn)) : 1.0f) * Math.pow(right_trigger_deadband(deadbandIn), power));}

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