package com.epra;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.HashMap;
import java.util.Map;

public class Controller extends Gamepad {
    Gamepad gamepad;
    public enum Key {
        A,
        B,
        X,
        Y,
        UP,
        DOWN,
        LEFT,
        RIGHT,
        BUMPER_LEFT,
        BUMPER_RIGHT,
        STICK_LEFT,
        STICK_RIGHT,
        LEFT_STICK_X,
        RIGHT_STICK_X,
        LEFT_STICK_Y,
        RIGHT_STICK_Y,
        LEFT_TRIGGER,
        RIGHT_TRIGGER;

        Key() {}
    }

    /**A map containing all of the buttons and corresponding keys.*/
    public Map<Key, Button> map = new HashMap<>();

    private float deadband;

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
     * @param deadbandIn The starting deadband range.
     * @param g The gamepad this controller instance will extend.
     * */
    public Controller(Gamepad g, float deadbandIn) {
        gamepad = g;
        deadband = deadbandIn;
        map.put(Key.A, new Button(gamepad.a));
        map.put(Key.B, new Button(gamepad.b));
        map.put(Key.X, new Button(gamepad.x));
        map.put(Key.Y, new Button(gamepad.y));
        map.put(Key.UP, new Button(gamepad.dpad_up));
        map.put(Key.DOWN, new Button(gamepad.dpad_down));
        map.put(Key.LEFT, new Button(gamepad.dpad_left));
        map.put(Key.RIGHT, new Button(gamepad.dpad_right));
        map.put(Key.BUMPER_LEFT, new Button(gamepad.left_bumper));
        map.put(Key.BUMPER_RIGHT, new Button(gamepad.right_bumper));
        map.put(Key.STICK_LEFT, new Button(gamepad.left_stick_button));
        map.put(Key.STICK_RIGHT, new Button(gamepad.right_stick_button));
        map.put(Key.LEFT_STICK_X, new Button(gamepad.left_stick_x));
        map.put(Key.RIGHT_STICK_X, new Button(gamepad.right_stick_x));
        map.put(Key.LEFT_STICK_Y, new Button(gamepad.left_stick_y));
        map.put(Key.RIGHT_STICK_Y, new Button(gamepad.right_stick_y));
    }
    /**Updates the button values in the map.*/
    public void update() {
        map.get(Key.A).update(gamepad.a);
        map.get(Key.B).update(gamepad.b);
        map.get(Key.X).update(gamepad.x);
        map.get(Key.Y).update(gamepad.y);
        map.get(Key.UP).update(gamepad.dpad_up);
        map.get(Key.DOWN).update(gamepad.dpad_down);
        map.get(Key.LEFT).update(gamepad.dpad_left);
        map.get(Key.RIGHT).update(gamepad.dpad_right);
        map.get(Key.BUMPER_LEFT).update(gamepad.left_bumper);
        map.get(Key.BUMPER_RIGHT).update(gamepad.right_bumper);
        map.get(Key.STICK_LEFT).update(gamepad.left_stick_button);
        map.get(Key.STICK_RIGHT).update(gamepad.right_stick_button);
        map.get(Key.LEFT_STICK_X).update(gamepad.left_stick_x);
        map.get(Key.RIGHT_STICK_X).update(gamepad.right_stick_x);
        map.get(Key.LEFT_STICK_Y).update(gamepad.left_stick_y);
        map.get(Key.RIGHT_STICK_Y).update(gamepad.right_stick_y);
    }

    /**Returns the float value of an analog.
     * @param analog Corresponding key for analog.*/
    public float getAnalog(Key analog) { return map.get(analog).toFloat(); }
    /**Returns the boolean value of a button.
     * @param button Corresponding key for button.*/
    public boolean getButton(Key button) { return map.get(button).toBoolean(); }
    /**Returns the value of a button and an int.
     * @param button Corresponding key for button.*/
    public int getButtonInt(Key button) { return boolToInt(getButton(button)); }

    /**Sets deadband limit for joysticks and triggers.
     * @param d Deadband range.*/
    public void setDeadband(float d) { deadband = d; }
    /**Returns deadband limit for joysticks and triggers.*/
    public float getDeadband() { return deadband; }
    /**Returns 0 if in the deadband range, if not returns as normal.
     * @param analog Corresponding key for analog.*/
    public float analogDeadband(Key analog) { return (Math.abs(map.get(analog).toFloat()) > deadband) ? map.get(analog).toFloat() : 0.0F; }
    /**Returns 0 if in the deadband range, if not returns as normal.
     * @param analog Corresponding key for analog.
     * @param deadbandIn Deadband range.*/
    public float analogDeadband(Key analog, float deadbandIn) { return (Math.abs(map.get(analog).toFloat()) > deadbandIn) ? map.get(analog).toFloat() : 0.0F; }
    /**Returns the value raised to the power of the input.
     * @param analog Corresponding key for analog.
     * @param power Power to be raised to.*/
    public float analogPower(Key analog, int power) { return Math.signum(map.get(analog).toFloat() * (float)Math.pow(Math.abs(map.get(analog).toFloat()), power)); }
    /**If the value is within the deadband range, it is set to 0. If not, it is raised to the power of the input.
     * @param analog Corresponding key for analog.
     * @param power to be raised to.*/
    public float analogPowerDeadband(Key analog, int power) { return (Math.abs(analogPower(analog, power)) > deadband) ? analogPower(analog, power) : 0.0F; }
    /**If the value is within the deadband range, it is set to 0. If not, it is raised to the power of the input.
     * @param analog Corresponding key for analog.
     * @param power Power to be raised to.
     * @param deadbandIn Deadband range.*/
    public float analogPowerDeadband(Key analog, int power, int deadbandIn) { return (Math.abs(analogPower(analog, power)) > deadbandIn) ? analogPower(analog, power) : 0.0F; }

    /**Returns a true output only on the first call while a button is pressed.
     * If the method is called again while the button is still pressed, the return will be false.
     * If the method is called while the button is released it will reset.
     * @param button Corresponding key for button.*/
    public boolean buttonSingle(Key button) {
        boolean r = false;
        if (map.get(button).toBoolean()) {
            if (!map.get(button).flag) {
                r = true;
                map.get(button).flag = true;
            }
        } else {
            map.get(button).flag = false;
        }
        return r;
    }
    /**Returns the output of buttonSingle as an int.
     * @param button Corresponding key for button.*/
    public int buttonSingleInt(Key button) {return boolToInt(buttonSingle(button));}
    /**Will change the state of the toggle if the button is pressed.
     * Returns the new state of the toggle.
     * @param button Corresponding key for button.*/
    public boolean buttonToggle(Key button) {
        if (map.get(button).toBoolean()) {
            map.get(button).toggle = !(map.get(button).toggle);
        }
        return map.get(button).toggle;
    }
    /**Returns the output of buttonToggle as an int.
     * @param button Corresponding key for button.*/
    public int buttonToggleInt(Key button) {return boolToInt(buttonToggle(button));}
    /**Will change the state of the toggle if the button is pressed following the rules of buttonSingle.
     * Returns the new state of the toggle.
     * @param button Corresponding key for button.*/
    public boolean buttonToggleSingle(Key button) {
        if (buttonSingle(button)) {
            map.get(button).toggle = !(map.get(button).toggle);
        }
        return map.get(button).toggle;
    }
    /**Returns the output of buttonToggleSingle as an int.
     * @param button Corresponding key for button.*/
    public int buttonToggleSingleInt(Key button) {return boolToInt(buttonToggleSingle(button));}
    /**Will change the state of the toggle regardless of the state of the button.
     * Returns the new state of the toggle.
     * @param button Corresponding key for button.*/
    public boolean flipToggle(Key button) {
        map.get(button).toggle = !(map.get(button).toggle);
        return map.get(button).toggle;
    }
    /**Returns the state of the toggle without changing the state of the toggle.
     * @param button Corresponding key for button.*/
    public boolean getToggle(Key button) {
        return map.get(button).toggle;
    }
    /**Returns the output of getToggle as an int.
     * @param button Corresponding key for button.*/
    public int getToggleInt(Key button) {return boolToInt(buttonToggleSingle(button));}
    /**If the counter is more than or equal to max it will be reset and return zero. If not, the counter will increase by one and return the result.
     * @param button Corresponding key for button.
     * @param max The maximum value of the counter.*/
    public int buttonCounter(Key button, int max) {
        if (map.get(button).toBoolean()) {
            map.get(button).counter = (map.get(button).counter + 1) % max;
        }
        return map.get(button).counter;
    }
    /**Will perform the same action as buttonCounter but follows the rules of buttonSingle.
     * @param button Corresponding key for button.
     * @param max The maximum value of the counter.*/
    public int buttonCounterSingle(Key button, int max) {
        if (buttonSingle(button)) {
            map.get(button).counter = (map.get(button).counter + 1) % max;
        }
        return map.get(button).counter;
    }
    /**Will increase the counter of a certain button by a certain amount. If the counter goes over max, it will reset and overflow. Returns the new value of the counter.
     * @param button Corresponding key for button.
     * @param max The maximum value of the counter.
     * @param increase The amount by which the counter will increase.*/
    public int increaseCounter(Key button, int max, int increase) {
        map.get(button).counter = (map.get(button).counter + increase + max) % max;
        return map.get(button).counter;
    }
    /**Will set the counter to a certain number.
     * @param button Corresponding key for button.
     * @param set The value to set the counter to.*/
    public void setCounter(Key button, int set) { map.get(button).counter = set; }
    /**Returns the current value of the counter.
     * @param button Corresponding key for button.*/
    public int getCounter(Key button) { return map.get(button).counter; }

    /**If true will return 1, if false will return 0.
     * @param b The input boolean.*/
    public int boolToInt(boolean b) {return (b) ? 1 : 0;}
}