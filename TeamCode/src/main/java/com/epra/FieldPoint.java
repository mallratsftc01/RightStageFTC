package com.epra;

public class FieldPoint {
    public float x;
    public float y;
    /**Stores a X, Y coordinate to indicate the robot's position on the field.
     *<p></p>
     *Queer Coded by Zachy K. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
    public FieldPoint (float x, float y) {
        this.x = x;
        this.y = y;
    }
    /**Sets the stored point to the provided values*/
    public void set (float x, float y) {
        this.x = x;
        this.y = y;
    }
}
