package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import java.util.ArrayList;

import java.util.List;

public class CameraPlus {
    public static final boolean USE_WEBCAM = true;
    private List<AprilTagDetection> currentDetections;
    private List<Recognition> currentRecognitions;
    private AprilTagProcessor aprilTag;
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    private int targetID;
    private int pixelCheckStep = 0;
    private int pixelReturn = 0;

    public CameraPlus (AprilTagProcessor aprilTag, TfodProcessor tfod, VisionPortal visionPortal) {
        this.aprilTag = aprilTag;
        this.visionPortal = visionPortal;
        this.tfod = tfod;
        currentDetections = new ArrayList<AprilTagDetection>();
        currentRecognitions = new ArrayList<Recognition>();
        updateDetections();
        updateRecognitions();
        startingTarget();
    }
    public void updateDetections () {
        if (aprilTag.getDetections().size() > 0) {
            currentDetections = aprilTag.getDetections();
        }
    }

    public void updateRecognitions() {
        if (tfod.getRecognitions().size() > 0) {
            currentRecognitions = tfod.getRecognitions();
        }
    }

    public int getID (int index) {
        updateDetections();
        return (index < currentDetections.size()) ? (currentDetections.get(index).metadata != null) ? currentDetections.get(index).id : -1 : -1;
    }

    public int getTargetID() {return targetID;}

    public void setTargetID(int t) {targetID = t;}
    public void startingTarget() {targetID = getID(0);}
    public void targetIDPlus(int t) {targetID += t;}

    public void targetIDMinus(int t) {targetID -= t;}

    public boolean atTargetID() {
        boolean r = false;
        updateDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && detection.id == targetID) {r = true;}
        }
        return r;
    }

    public int getTotalDetections () {
        updateDetections();
        return currentDetections.size();
    }

    public String getName (int index) {
        updateDetections();
        return (index < currentDetections.size()) ? (currentDetections.get(index).metadata != null) ? currentDetections.get(index).metadata.name : "Index too high" : "Null";
    }

    public double getY(int index) {
        updateDetections();
        return (index < currentDetections.size()) ? (currentDetections.get(index).metadata != null) ? currentDetections.get(index).ftcPose.y : -1 : -1;
    }

    public double getX(int index) {
        updateDetections();
        return (index < currentDetections.size()) ? (currentDetections.get(index).metadata != null) ? currentDetections.get(index).ftcPose.x : -1 : -1;
    }

    public double getYaw(int index) {
        updateDetections();
        return (index < currentDetections.size()) ? (currentDetections.get(index).metadata != null) ? currentDetections.get(index).ftcPose.yaw : -1 : -1;
    }

    public String getLabel (int index) {
        updateRecognitions();
        return (index < currentRecognitions.size()) ?  currentRecognitions.get(index).getLabel() : "index too high";
    }

    public int numRecognitions () {
        return currentRecognitions.size();
    }

    public int pixelPosition (int checksPerPos) {
        //changes the step if appropriate, or ends the program
        if (pixelCheckStep % checksPerPos == 0) {
            switch (pixelCheckStep / checksPerPos) {
                case 0:
                    tfod.setClippingMargins(640, 480, 640, 480);
                    break;
                case 1:
                    tfod.setClippingMargins(480, 360, 0, 0);
                    pixelReturn = 3;
                    break;
                case 2:
                    tfod.setClippingMargins(160, 360, 160, 0);
                    pixelReturn = 2;
                    break;
                case 3:
                    tfod.setClippingMargins(200, 0, 0, 0);
                    pixelCheckStep = -1;
                    pixelReturn = 1;
                    break;
            }
        }
        //checks to see if any pixels are seen from this view
        if (pixelCheckStep >= checksPerPos && pixelCheckStep < checksPerPos * 3) {
            pixelReturn = (numRecognitions() > 0) ? pixelReturn : 0;
        }
        pixelCheckStep++;
        return pixelReturn;
    }

    public int getPixelCheckStep() {return pixelCheckStep;}
}
