package com.epra.location;

import com.epra.storage.IMUStorage;
import com.epra.IMUExpanded;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTagLocator {
    private final FieldPoint[] ID = new FieldPoint[]  {
            new FieldPoint(0.0f, 0.0f), //Origin
            new FieldPoint (115.25f, 137.0f),
            new FieldPoint (109.25f, 137.0f),
            new FieldPoint (103.25f, 137.0f),
            new FieldPoint (40.75f, 137.0f),
            new FieldPoint (34.75f, 137.0f),
            new FieldPoint (28.75f, 137.0f),
            new FieldPoint (31.0f, 0.0f),
            new FieldPoint (37.0f, 0.0f),
            new FieldPoint (107.0f, 0.0f),
            new FieldPoint (113.0f, 0.0f)
    };

    private List<AprilTagDetection> currentDetections;
    private AprilTagProcessor aprilTagProcessor;
    private FieldPoint location;
    int age;

    /**Uses an AprilTagProcessor in order to estimate the location of the robot.
     *<p></p>
     *Queer Coded by Zachy K. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
    public AprilTagLocator (AprilTagProcessor aprilTagProcessor) {
        this.aprilTagProcessor = aprilTagProcessor;
        location = new FieldPoint(0.0f, 0.0f);
        age = 0;
    }

    /**Updates the detections of aprilTags only if the detections are fresh.
     * Returns true if the detections are fresh and false if they are not*/
    public boolean updateDetections () {
        if (aprilTagProcessor.getFreshDetections() != null) {
            currentDetections = aprilTagProcessor.getFreshDetections();
            age = 0;
            return true;
        } else {
            age++;
            return false;
        }
    }
    /**Returns the age of the latest detection*/
    public int getAge () {return age;}
    /**Returns true if any detected tag's id matches the parameterized id.*/
    public boolean findTag (int tagID) {
        if (updateDetections()) {
            boolean b = false;
            if (currentDetections != null) {
                for (AprilTagDetection a : currentDetections) {
                    if (a.id == tagID) {
                        b = true;
                        break;
                    }
                }
                return b;
            } else {
                return false;
            }
        } else {
            return false;
        }
    }
    /**Updates the location of the robot by using the AprilTagProcessor and IMUStorage only if the detections are fresh.
     * Returns true if the detections are fresh and false if they are not */
    public boolean updateLocation(IMUStorage imu) {
        if (updateDetections()) {
            FieldPoint p;
            location.x = 0.0f;
            location.y = 0.0f;
            for (int ii = 0; ii < currentDetections.size(); ii++) {
                p = ID[currentDetections.get(ii).id];
                double hyp = Math.sqrt(Math.pow(currentDetections.get(ii).ftcPose.x, 2) + Math.pow(currentDetections.get(ii).ftcPose.y, 2));
                double rot = (imu.avgIMU(IMUExpanded.YAW, AngleUnit.DEGREES) % 90) - currentDetections.get(ii).ftcPose.yaw;
                p.y += Math.signum(p.y * -1) * Math.abs(Math.cos(rot) * hyp);
                p.x += (Math.signum(currentDetections.get(ii).ftcPose.x) * ((currentDetections.get(ii).id < 7) ? -1 : 1) * Math.abs(Math.sin(rot) * hyp));
                location.x += p.x;
                location.y += p.y;
            }
            location.x /= currentDetections.size();
            location.y /= currentDetections.size();
            return true;
        } else {
            return false;
        }
    }
    /**Updates the location and returns the latest location in FieldPoint form*/
    public FieldPoint getLocation (IMUStorage imu) {
        updateLocation(imu);
        return location;
    }

}
