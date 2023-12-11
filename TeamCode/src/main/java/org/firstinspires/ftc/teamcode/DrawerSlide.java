package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class DrawerSlide {
    final double WORM_CIRC = 0.254 * 3.14;
    final double WORM_GEAR_RATIO = 72.0 / 60.0 / 28.0;
    final double SHOULDER_GEAR_RATIO = 1.0/28.0;
    final double SHOULDER_HIGHT = 11.0;
    final double ARM_LENGTH = 17.5;
    DcMotorEx shoulder;
    DcMotorEx extend;
    Servo wrist;
    Servo pinch;
    TouchSensor magnet;

    int shoulderStart;
    int extendStart;

    /**Queer Coded by ZK. If you use this class or a method from this class in its entirety, please make sure to give credit.
     * <p></p>
     * Introduces methods useful for the implementation of a drawer slide based arm system.*/
    public DrawerSlide(DcMotorEx shoulder, DcMotorEx extend, Servo wrist, Servo pinch, TouchSensor magnet) {
        this.shoulder = shoulder;
        this.extend = extend;
        this.wrist = wrist;
        this.pinch = pinch;
        this.magnet = magnet;

        shoulderStart = shoulder.getCurrentPosition();
        extendStart = extend.getCurrentPosition();
    }
    /**Returns the current position of the shoulder motor.*/
    public int getShoulderPos() {return shoulder.getCurrentPosition();}
    /**Returns the current distance of the shoulder motor from the starting point.*/
    public int getShoulderDist() {return shoulder.getCurrentPosition() - shoulderStart;}
    /**Returns the current position of the extend motor.*/
    public int getExtendPos() {return extend.getCurrentPosition();}
    /**Returns the current distance of the extend motor from the starting point.*/
    public int getExtendDist() {return extend.getCurrentPosition() - extendStart;}

    /**Returns the inches extended by the extend motor.*/
    public double getExtendInch() {return Math.abs(getExtendDist() * WORM_GEAR_RATIO * WORM_CIRC);}

    /**Returns the height of the end of the arm in inches from the ground.*/
    public double getClawHeightInch() {
        double angle = (getShoulderDist() * SHOULDER_GEAR_RATIO * 360) - 30;
        return ((getExtendInch() + ARM_LENGTH) / Math.sin(angle)) + SHOULDER_HIGHT;
    }

    /**Returns the current position of the wrist servo.*/
    public double getWristPos() {return wrist.getPosition();}
    /**Returns the current position of the pinch servo.*/
    public double getPinchPos() {return pinch.getPosition();}

    /**Returns the current state of the magnet sensor.*/
    public boolean getMagnet() {return magnet.isPressed();}

    /**Using the current position of the shoulder motor and the input target will calculate a power double.*/
    public double targetShoulderPos(int target) {
        int dist = target - shoulder.getCurrentPosition();
        double pow = Math.signum(dist);
        pow *= (Math.abs(dist) <= 100) ? (dist/100) : 0;
        pow *= (pow < 0.1) ? 0 : 1;
        return pow;
    }
    /**Using the current position of the extend motor and the input target will calculate a power double.*/
    public double targetExtendPos(int target) {
        int dist = target - extend.getCurrentPosition();
        double pow = Math.signum(dist);
        pow *= (Math.abs(dist) <= 100) ? (dist/100) : 0;
        pow *= (pow < 0.1) ? 0 : 1;
        return pow;
    }
    /**Using the current distance of the shoulder motor from the starting position and the input target will calculate a power double.*/
    public double targetShoulderDist(int target) {
        int dist = target - getShoulderDist();
        double pow = Math.signum(dist);
        pow *= (Math.abs(dist) <= 100) ? (dist/100) : 0;
        pow *= (pow < 0.1) ? 0 : 1;
        return pow;
    }
    /**Using the current distance of the extend motor from the starting position and the input target will calculate a power double.*/
    public double targetExtendDist(int target) {
        int dist = target - getExtendDist();
        double pow = Math.signum(dist);
        pow *= (Math.abs(dist) <= 100) ? (dist/100) : 0;
        pow *= (pow < 0.1) ? 0 : 1;
        return pow;
    }
    /**Will convert inches into encoder ticks and use targetExtendDist to convert that int into a power.*/
    public double targetExtendInch(double target) {
        int posTarget = (int)(target / WORM_GEAR_RATIO / WORM_CIRC);
        return targetExtendDist(posTarget);
    }

    /**Will set the shoulder motor's power to the input.*/
    public void moveShoulder(double pow) {shoulder.setPower(pow);}
    /**Will set the extend motor's power to the input.*/
    public void moveExtend(double pow) {extend.setPower(pow);}

    /**Will set the extend motor's power to the first input if the magnet isn't pressed and to the second input if it is.*/
    public boolean moveExtendMagnet(double pow, double elsePow) {
        extend.setPower((getMagnet()) ? elsePow : pow);
        return getMagnet();
    }

    /**Will set the position of the wrist servo to the input.*/
    public void setWrist(double pos) {wrist.setPosition(pos);}
    /**Will set the position of the pinch servo to the input.*/
    public void setPinch(double pos) {pinch.setPosition(pos);}
}
