package com.epra;

import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.util.ArrayList;

public class IMUExpanded{
    static public final int YAW = 0;
    static public final int PITCH = 1;
    static public final int ROLL = 2;
    ArrayList<IMU> imus = new ArrayList<IMU>();
    /**Increases the functionality of the IMU class.
     *<p></p>
     *Queer Coded by ZK. If you use this class or a method from this class in its entirety, please make sure to give credit.
     *<p></p>
     *Expands the functionality of one IMU.*/
    public IMUExpanded(IMU imu) {
        imus.add(imu);
    }
    /**Increases the functionality of the IMU class.
     *<p></p>
     *Queer Coded by ZK. If you use this class or a method from this class in its entirety, please make sure to give credit.
     *<p></p>
     *Expands the functionality of two IMUs.*/
    public IMUExpanded(IMU imu1, IMU imu2) {
        imus.add(imu1);
        imus.add(imu2);
    }
    /**Increases the functionality of the IMU class.
     *<p></p>
     *Queer Coded by ZK. If you use this class or a method from this class in its entirety, please make sure to give credit.
     *<p></p>
     *Expands the functionality of one or more IMUs.*/
    public IMUExpanded(IMU[] imu) {
        for (IMU e : imu) {imus.add(e);}
    }

    /**Returns the average orientation of the IMU(s).*/
    public double avgIMU(int axis, AngleUnit angleUnit) {
        double r = 0;
        for (int ii = 0; ii < imus.size(); ii++) {
            switch (axis) {
                case 0:
                    r += imus.get(ii).getRobotYawPitchRollAngles().getYaw(angleUnit);
                    break;
                case 1:
                    r += imus.get(ii).getRobotYawPitchRollAngles().getPitch(angleUnit);
                    break;
                case 2:
                    r += imus.get(ii).getRobotYawPitchRollAngles().getRoll(angleUnit);
                    break;
            }
        }
        return r / imus.size();
    }
    /**Returns the distance between the current orientation of the IMU(s) and the target. Do not use, always use trueDistIMU.*/
    public double distIMU(int axis, AngleUnit angleUnit, double target) {return target - avgIMU(axis, angleUnit);}
    /**Returns the true distance between the orientation of the IMU(s) and the target, including looping from 360 to 1.*/
    public double trueDistIMU(int axis, AngleUnit angleUnit, double target) {
        double current = avgIMU(axis, angleUnit) + 180;
        double newTarget = target;
        if (Math.min(target, current) == target) {newTarget += 360;}
        else {current += 360;}
        return Math.min(distIMU(axis,angleUnit, target), (newTarget - current));
    }
}
