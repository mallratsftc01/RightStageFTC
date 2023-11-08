package com.epra;

public class FloatControl {
    private float value = 0.0F;
    private float deadband;
    public FloatControl (float d) {
        //the deadband stores a value that when the the joystick is under or over the negative equivalent only 0 will be returned
        deadband = d;
    }

    public void setValue (float inVal) {
        value = inVal;
    }

    public float getValue () {
        return value;
    }
    //method for deadbanding
    public float getDeadband () {
        return (value > deadband && value < -1.0 * deadband) ? value : 0.0F;
    }
    //method for squaring the float
    public float getSquared () {
        return getSign() * (value * value);
    }

    public float getSquaredDeadband () {
        return (getSquared() > deadband && getSquared() < -1.0 * deadband) ? getSquared() : 0.0F;
    }
    public float getSign () {
        return Math.signum(value);
    }
}
