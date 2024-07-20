package com.epra.location;

import com.epra.storage.IMUStorage;
import com.epra.IMUExpanded;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTagLocator {

    public AprilTagLocator() {

    }

    /**
     * Calculates the location of the robot relative to an AprilTag.
     * @param x X of a detection pose.
     * @param y Y of a detection pose.
     * @param z Z of a detection pose.
     * @param c Distance from the camera to center of robot.
     * @param yaw Yaw angle of the robot relative to the AprilTag in radians.
     * @param pitch Pitch angle of the robot relative to the AprilTag in radians.
     * @return A double array of length 2 with [x, y] coordinates of the robot relative to the AprilTag
     */
    public double[] relativeLocation (double x, double y, double z, double c, double yaw, double pitch) {
        double[] ret = new double[3];
        double hypXY = Math.sqrt((x * x) + ((y + c) * (y + c)));
        double hypYZ = Math.sqrt((z * z) + ((y + c) * (y + c)));
        ret[0] = Math.sin(2 * yaw) * hypXY;
        ret[1] = ((Math.cos(2 * yaw) * hypXY)) /* + (Math.cos(2 * pitch) * hypYZ)) / 2*/;
        ret[2] = Math.sin(2 * pitch) * hypYZ;
        return ret;
    }

    /**
     * Calculates the polar coordinates of the robot with an April Tag as the origin
     * @param x X of a detection pose.
     * @param z Z of a detection pose.
     * @param c Distance from the camera to gyro location.
     * @param a Angle of the robot in radians.
     * @return A double array of length 2 with [radius, theta] coordinates.
     */
    public double[] relativeLocationPolar (double x, double z, double c, double a) {
        double[] ret = new double[2];
        ret[0] = Math.sqrt(((c + z) * (c + z)) + (x * x));
        ret[1] = ((Math.PI / 2) - a) + Math.atan((c + z) / x);
        return ret;
    }

    /**
     * Converts from polar coordinates to cartesian coordinates.
     * @param polar Polar coordinates in the form [radius, theta].
     * @return A double array of length 2 with [x, y].
     */
    public double[] toCartesian (double[] polar) {
        double ret[] = new double[2];
        ret[0] = polar[0] * Math.cos(polar[1]);
        ret[1] = polar[0] * Math.sin(polar[1]);
        return ret;
    }
}
