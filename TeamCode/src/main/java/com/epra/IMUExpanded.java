package com.epra;

import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.util.ArrayList;

public class IMUExpanded {
    public final int YAW = 0;
    public final int PITCH = 1;
    public final int ROLL = 2;
    ArrayList<IMU> imus = new ArrayList<IMU>();
    public IMUExpanded(IMU imu) {
        imus.add(imu);
    }

    public IMUExpanded(IMU imu1, IMU imu2) {
        imus.add(imu1);
        imus.add(imu2);
    }

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
        double current = avgIMU(axis, angleUnit);
        if (Math.min(target, current) == target) {target += 360;}
        else {current += 360;}
        return Math.min(distIMU(axis,angleUnit, target), (target - current));
    }
}
