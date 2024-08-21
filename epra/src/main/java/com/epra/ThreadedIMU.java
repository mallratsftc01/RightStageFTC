package com.epra;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
public class ThreadedIMU implements Runnable{
    IMU imu;
    AngleUnit defaultAngleUnit = AngleUnit.DEGREES;
    double yaw;
    double pitch;
    double roll;
    int ii = 0;
    public ThreadedIMU(IMU imuIn) {
        imu = imuIn;
        IMU.Parameters perry = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(perry);
        yaw = imu.getRobotYawPitchRollAngles().getYaw(defaultAngleUnit);
        pitch = imu.getRobotYawPitchRollAngles().getPitch(defaultAngleUnit);
        roll = imu.getRobotYawPitchRollAngles().getRoll(defaultAngleUnit);
    }

    public void run() {
        while (1==1) {
            ii++;
            yaw = imu.getRobotYawPitchRollAngles().getYaw(defaultAngleUnit);
            pitch = imu.getRobotYawPitchRollAngles().getPitch(defaultAngleUnit);
            roll = imu.getRobotYawPitchRollAngles().getRoll(defaultAngleUnit);
        }
    }

    public double getYaw() {return yaw;}
    public double getPitch() {return pitch;}
    public double getRoll() {return roll;}
    public int getII() {return ii;}
}
