package com.epra;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.CameraPlus;

public class LocationServices {
    private final float ID_6_X = 28.75f;
    private final float ID_5_X = 34.75f;
    private final float ID_4_X = 40.75f;
    private final float ID_3_X = 103.25f;
    private final float ID_2_X = 109.25f;
    private final float ID_1_X = 115.25f;
    private final float ID_10_X = 113.0f;
    private final float ID_9_X = 107.0f;
    private final float ID_8_X = 37.0f;
    private final float ID_7_X = 31.0f;
    private final float ID_UNDER_7_Y = 137.0f;
    private float x;
    private float y;
    private DcMotorEx motor0;
    private DcMotorEx motor1;
    private DcMotorEx motor2;
    private DcMotorEx motor3;
    int motor0Last;
    int motor1Last;
    int motor2Last;
    int motor3Last;
    float yRetention;
    float xRetention;
    private Controller controller1;
    private CameraPlus cam;
    float wheelCirc;
    IMU imu;
    private float saveX;
    private float saveY;

    public LocationServices(float xStart, float yStart, DcMotorEx motor0In, DcMotorEx motor1In, DcMotorEx motor2In, DcMotorEx motor3In, float yRetentionIn, float xRetentionIn, Controller controllerIn, CameraPlus camIn, float wheelCircIn, IMU imuIn) {
        x = xStart;
        y = yStart;
        saveX = 0;
        saveY = 0;
        motor0 = motor0In;
        motor1 = motor1In;
        motor2 = motor2In;
        motor3 = motor3In;
        yRetention = yRetentionIn;
        xRetention = xRetentionIn;
        controller1 = controllerIn;
        cam = camIn;
        wheelCirc = wheelCircIn;
        imu = imuIn;
    }

    public void setYRetention(float rotIn) {yRetention = rotIn;};
    public void setXRetention(float rotIn) {xRetention = rotIn;};
    public void updateLast() {
        motor0Last = motor0.getCurrentPosition();
        motor1Last = motor1.getCurrentPosition();
        motor2Last = motor2.getCurrentPosition();
        motor3Last = motor3.getCurrentPosition();
    }

    public float getX(){return x;}
    public float getY(){return y;}

    public int distTraveledMotor0() {return Math.abs(motor0.getCurrentPosition()) - Math.abs(motor0Last);}
    public int distTraveledMotor1() {return Math.abs(motor1.getCurrentPosition()) - Math.abs(motor1Last);}
    public int distTraveledMotor2() {return Math.abs(motor2.getCurrentPosition()) - Math.abs(motor2Last);}
    public int distTraveledMotor3() {return Math.abs(motor3.getCurrentPosition()) - Math.abs(motor3Last);}

    public float movementXRobotRelative() {return (distTraveledMotor0() + distTraveledMotor1() + distTraveledMotor2() + distTraveledMotor3() / 4.0f) * wheelCirc * xRetention * controller1.left_stick_x;}
    public float movementYRobotRelative() {return (distTraveledMotor0() + distTraveledMotor1() + distTraveledMotor2() + distTraveledMotor3() / 4.0f) * wheelCirc * yRetention * controller1.left_stick_y;}

    public void updatePositionWithMotors() {
        double rot = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) % 90;
        x += Math.sin(rot) * movementYRobotRelative();
        y += Math.cos(rot) * movementYRobotRelative();
        x += Math.cos(rot) * movementXRobotRelative();
        y += Math.sin(rot) * movementXRobotRelative();
        updateLast();
    }

    public void updatePositionWithAprilTag() {
        float tempX = 0;
        float tempY = 0;
        tempY = (cam.getID(0) < 7) ? ID_UNDER_7_Y : 0;
        switch (cam.getID(0)) {
            case 1:
                tempX = ID_1_X;
                break;
            case 2:
                tempX = ID_2_X;
                break;
            case 3:
                tempX = ID_3_X;
                break;
            case 4:
                tempX = ID_4_X;
                break;
            case 5:
                tempX = ID_5_X;
                break;
            case 6:
                tempX = ID_6_X;
                break;
            case 7:
                tempX = ID_7_X;
                break;
            case 8:
                tempX = ID_8_X;
                break;
            case 9:
                tempX = ID_9_X;
                break;
            case 10:
                tempX = ID_10_X;
                break;
        }
        double hyp = Math.sqrt((cam.getX(0) * cam.getX(0)) + (cam.getY(0) * cam.getY(0)));
        double rot = (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) % 90) - cam.getYaw(0);
        tempY += (float)(((tempY > 0) ? -1 : 1) * Math.abs(Math.cos(rot) * hyp));
        tempX += (float)(Math.signum(cam.getX(0)) * ((cam.getID(0) < 7) ? -1 : 1) * Math.abs(Math.sin(rot) * hyp));
        if (tempX != saveX && tempY != saveY) {
            x = tempX;
            y = tempY;
        } else {
            updatePositionWithMotors();
        }
        updateLast();
    }

    public void updatePositionGeneral() {
        if (cam.getID(0) > -1) {
            updatePositionWithAprilTag();
        } else {
            updatePositionWithMotors();
        }
    }
}
