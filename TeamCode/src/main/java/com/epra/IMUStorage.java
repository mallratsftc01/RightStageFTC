package com.epra;

import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.util.ArrayList;
public class IMUStorage{
    double[][] imuValues = new double[2][3];

    /**Stores values for the IMU and uses them to perform calculations.
     *<p></p>
     *Queer Coded by Zachy K. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
    public IMUStorage (double yawDegrees, double pitchDegrees, double rollDegrees, double yawRadians, double pitchRadians, double rollRadians) {
        updateIMUValues(yawDegrees,  pitchDegrees,  rollDegrees, yawRadians, pitchRadians, rollRadians);}

    /**Stores values provided.*/
    public void updateIMUValues(double yawDegrees, double pitchDegrees, double rollDegrees, double yawRadians, double pitchRadians, double rollRadians) {
        imuValues[0][0] = yawDegrees;
        imuValues[0][1] = pitchDegrees;
        imuValues[0][2] = rollDegrees;
        imuValues[1][0] = yawRadians;
        imuValues[1][1] = pitchRadians;
        imuValues[1][2] = rollRadians;
    }
    /**Returns the average orientation of the IMU(s) based on stored values.*/
    public double avgIMU(int axis, AngleUnit angleUnit) {
        if (angleUnit.equals(AngleUnit.DEGREES)) {return imuValues[0][axis];}
        if (angleUnit.equals(AngleUnit.RADIANS)) {return imuValues[1][axis];}
        else {return 0;}
    }
    /**Returns the distance between the current orientation of the IMU(s) and the target based on stored values. Do not use, always use trueDistIMU.*/
    public double distIMU(int axis, AngleUnit angleUnit, double target) {return target - avgIMU(axis, angleUnit);}
    /**Returns the true distance between the orientation of the IMU(s) and the target based on stored values, including looping from 360 to 1.*/
    public double trueDistIMU(int axis, AngleUnit angleUnit, double target) {
        double current = avgIMU(axis, angleUnit);
        double newTarget = target;
        if (Math.min(target, current) == target) {newTarget += 360;}
        else {current += 360;}
        double dist1 = Math.abs(distIMU(axis,angleUnit, target));
        double r =(Math.min(dist1, Math.abs((newTarget - current))) == dist1) ? distIMU(axis,angleUnit, target) : (newTarget - current);
        return (Math.abs(r) - 180) * Math.signum(r) * -1.0;
    }
}
