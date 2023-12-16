package com.epra.storage;

public class SensorIntStorage {
    private int[] encoderValues = new int[0];
    /**Stores values from sensors in integer form.
     *<p></p>
     *Queer Coded by Zachy K. If you use this class or a method from this class in its entirety, please make sure to give credit.
     *<p></p>
     *Intended to store values from motor encoders. */
    public SensorIntStorage(int[] startValues) {setEncoderValues(startValues);}

    /**Sets the stored values to match an array.*/
    public void setEncoderValues(int[] setValues) {
        if (encoderValues.length != setValues.length) {encoderValues = new int[setValues.length];}
        for (int ii = 0; ii < setValues.length; ii++) {encoderValues[ii] = setValues[ii];}
    }
    /**Sets the value of a certain int in the stored array. If provided index is out of range, will return false.*/
    public boolean setEncoderValue(int value, int index) {
        if (index < encoderValues.length) {encoderValues[index] = value;
    return true;} else {return false;}
    }
    /**Returns the stored int array.*/
    public int[] getEncoderValues() {return encoderValues;}
    /**Returns a certain value from the stored array if successful. If provided index is out of range, will return false.*/
    public int getEncoderValue(int index) {if (index < encoderValues.length) {return encoderValues[index];}
        else {return 0;}
    }
}
