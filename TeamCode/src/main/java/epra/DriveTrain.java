package epra;

import androidx.annotation.NonNull;

import epra.math.geometry.Geometry;
import epra.math.geometry.Vector;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.Set;

import java.util.HashMap;
import java.util.Map;

import epra.math.geometry.Angle;

/**
 * Queer Coded by Zachy K. If you use this class or a method from this class in its entirety, please make sure to give credit.
 * <p></p>
 * Coordinates motors in order to create cohesive robot motion.
 * This class can be used for a variable number of motors for several drive types.*/
public class DriveTrain {
    /**All the orientations a motor can be in relative to the DriveTrain.*/
    public static enum Orientation {
        RIGHT (new Angle(0.0)),
        LEFT (new Angle(0.0)),
        FRONT (new Angle(90.0)),
        BACK (new Angle(90.0)),
        RIGHT_FRONT (new Angle(315.0)),
        RIGHT_BACK (new Angle(45.0)),
        LEFT_FRONT (new Angle(45.0)),
        LEFT_BACK (new Angle(315.0));

        Angle angle;

        Orientation(Angle a) { angle = a;};

        /**
         * @param o Orientation to be tested.
         * @return True if the provided is right, false if not.
         */
        static boolean ifRight(Orientation o) {
            return switch (o) {
                case RIGHT, RIGHT_BACK, RIGHT_FRONT -> true;
                default -> false;
            };
        }

        /**
         * @param o Orientation to be tested.
         * @return True if the provided is left, false if not.
         */
        static boolean ifLeft(Orientation o) {
            return switch (o) {
                case LEFT, LEFT_BACK, LEFT_FRONT -> true;
                default -> false;
            };
        }

        /**
         * @param o Orientation to be tested.
         * @return True if the provided is front, false if not.
         */
        static boolean ifFront(Orientation o) {
            return switch (o) {
                case FRONT, RIGHT_FRONT, LEFT_FRONT -> true;
                default -> false;
            };
        }

        /**
         * @param o Orientation to be tested.
         * @return True if the provided is back, false if not.
         */
        static boolean ifBack(Orientation o) {
            return switch (o) {
                case BACK, LEFT_BACK, RIGHT_BACK -> true;
                default -> false;
            };
        }
    }

    /**All of the drive types currently available.*/
    public static enum DriveType {
        TANK,
        ARCADE,
        ZACHARIAN,
        X,
        MECANUM,

    }

    /**
     * Map of all the motors.
     */
    private Map<String, DcMotorEx> motor = new HashMap<>();
    /**
     * Map of all the motor powers.
     */
    private Map<String, Double> power = new HashMap<>();
    /**
     * Map of all motor positions.
     */
    private Map<String, Integer> pos = new HashMap<>();
    /**
     * Map of all orientations of motors.
     */
    private Map<String, Orientation> orientation = new HashMap<>();

    private DriveType driveType;
    private float wheelCircumference = 11.2192926146f; // 96mm diameter wheels circumference in inches
    private float gearRatio = 20;

    private Double targetDegrees = 0.0;

    /**
     * Queer Coded by Zachy K. If you use this class or a method from this class in its entirety, please make sure to give credit.
     * <p></p>
     * Coordinates motors in order to create cohesive robot motion.
     * This class can be used for a variable number of motors for several drive types.
     *
     * @param motorNames   Names that will be associated with each motor. Defaults to tank drive.
     * @param motors       DcMotorExs that will be used by the DriveTrain.
     * @param orientations The orientation of each motor.
     */
    public DriveTrain(String[] motorNames, DcMotorEx[] motors, Orientation[] orientations) {
        for (int i = 0; i < Math.min(motorNames.length, motors.length); i++) {
            motor.put(motorNames[i], motors[i]);
            power.put(motorNames[i], 0.0);
            pos.put(motorNames[i], 0);
            orientation.put(motorNames[i], orientations[i]);
        }
        driveType = DriveType.TANK;
    }

    /**
     * Queer Coded by Zachy K. If you use this class or a method from this class in its entirety, please make sure to give credit.
     * <p></p>
     * Coordinates motors in order to create cohesive robot motion.
     * This class can be used for a variable number of motors for several drive types.
     *
     * @param motorNames   Names that will be associated with each motor.
     * @param motors       DcMotorExs that will be used by the DriveTrain.
     * @param orientations The orientation of each motor.
     * @param driveTypeIn  The drive type to be used.
     */
    public DriveTrain(String[] motorNames, DcMotorEx[] motors, Orientation[] orientations, DriveType driveTypeIn) {
        for (int i = 0; i < Math.min(motorNames.length, motors.length); i++) {
            motor.put(motorNames[i], motors[i]);
            power.put(motorNames[i], 0.0);
            pos.put(motorNames[i], 0);
            orientation.put(motorNames[i], orientations[i]);
        }
        driveType = driveTypeIn;
    }

    /**
     * Right power directly powers right motors, left power directly powers left motors.
     *
     * @param powerRight Power for right motors.
     * @param powerLeft  Power for left motors.
     */
    public void tankDrive(float powerRight, float powerLeft) {
        for (Map.Entry<String, Orientation> entry : orientation.entrySet()) {
            if (Orientation.ifRight(entry.getValue())) {
                power.replace(entry.getKey(), (double) powerRight);
            } else if (Orientation.ifLeft(entry.getValue())) {
                power.replace(entry.getKey(), (double) powerLeft);
            }
        }
        setMotorPowers();
    }

    /**
     * Drive with only one joystick. Forward and backwards move forward and backwards, left and right turns.
     *
     * @param powerX The X position of the joystick.
     * @param powerY The Y position of the joystick.
     */
    public void arcadeDrive(float powerX, float powerY) {
        float powerRight = powerY + powerX;
        float powerLeft = powerY - powerX;
        powerLeft = (powerY > -0.5 && powerX < 0.5) ? 0 : powerLeft;
        powerRight = (powerX > -0.1 && powerY < 0.1) ? powerLeft : powerRight;
        tankDrive(powerRight, powerLeft);
    }

    /**
     * A joke drive created by accident while trying to create the arcade drive. Created 2/12/2022.
     * @param powerRightX X position of the right joystick.
     * @param powerLeftX  X position of the left joystick.
     * @param powerRightY Y position of the right joystick.
     * @param powerLeftY  Y position of the left joystick.
     */
    public void zacharianDrive(float powerRightX, float powerLeftX, float powerRightY, float powerLeftY) {
        float powerRight = powerRightY + powerRightX;
        float powerLeft = powerLeftY - powerLeftX;
        tankDrive(powerRight, powerLeft);
    }

    /**
     * Holonomic drive with four omni wheels. Created 9/17/2022.
     * @param powerRightX X position of the right joystick.
     * @param powerLeftX  X position of the left joystick.
     * @param powerLeftY  Y position of the left joystick.
     */
    public void xDrive(float powerRightX, float powerLeftX, float powerLeftY) {
        double powerVar = powerRightX * 0.25;
        for (Map.Entry<String, Orientation> entry : orientation.entrySet()) {
            if (Orientation.ifRight(entry.getValue())) {
                power.replace(entry.getKey(), (double) powerLeftY + powerVar);
            } else if (Orientation.ifLeft(entry.getValue())) {
                power.replace(entry.getKey(), (double) powerLeftX - powerVar);
            } else if (Orientation.ifFront(entry.getValue())) {
                power.replace(entry.getKey(), (double) powerLeftX + powerVar);
            } else if (Orientation.ifBack(entry.getValue())) {
                power.replace(entry.getKey(), (double) powerLeftY - powerVar);
            }
        }
        setMotorPowers();
    }

    /**
     * Holonomic drive with mecanum wheels. Left stick moves the robot, right stick X rotates the robot. Created 9/24/2022.
     * @param powerRightX X position of the right joystick.
     * @param powerLeftX  X position of the left joystick.
     * @param powerLeftY  Y position of the left joystick.
     */
    public void mecanumDrive(float powerRightX, float powerLeftX, float powerLeftY) {
        double denominator = Math.max(Math.abs(powerLeftY) + Math.abs(powerLeftX) + Math.abs(powerRightX), 1);
        for (Map.Entry<String, Orientation> entry : orientation.entrySet()) {
            switch (entry.getValue()) {
                case RIGHT_FRONT ->
                        power.replace(entry.getKey(), (double) (-1 * powerLeftY + powerRightX + powerLeftX) / denominator);
                case LEFT_FRONT ->
                        power.replace(entry.getKey(), (double) (-1 * powerLeftY + powerRightX - powerLeftX) / denominator);
                case RIGHT_BACK ->
                        power.replace(entry.getKey(), (double) (-1 * powerLeftY - powerRightX + powerLeftX) / denominator);
                case LEFT_BACK ->
                        power.replace(entry.getKey(), (double) (-1 * powerLeftY - powerRightX - powerLeftX) / denominator);
            }
        }
        setMotorPowers();
    }
    /**
     * Holonomic drive with mecanum wheels. Left stick moves the robot, right stick X rotates the robot. Uses the IMU to facilitate more accurate turns. Created 11/22/2023.
     * @param powerLeftX X position of the left joystick.
     * @param powerLeftY Y position of the left joystick.
     * @param vectorRight A vector representing the right joystick.
     * @param imu IMU to find angles and use methods.
     */
    public double[] gyroMecanumDrive(float powerLeftX, float powerLeftY, Vector vectorRight, IMUExpanded imu) {
        Angle current = imu.getYaw();
        float rightPow = (float) (Geometry.direction(current, vectorRight) * Math.min(Geometry.subtract(current, vectorRight).getRadian(), 1.0f) * vectorRight.getLength());
        mecanumDrive(rightPow, powerLeftX, powerLeftY);
        return new double[] {rightPow, Geometry.direction(current, vectorRight), Geometry.subtract(current, vectorRight).getRadian()};
    }

    /**Uses a drive based on the DriveTrain's drive type.
     * @param powerRightX X position of the right joystick.
     * @param powerLeftX  X position of the left joystick.
     * @param powerRightY Y position of the right joystick.
     * @param powerLeftY  Y position of the left joystick.
     * */
    public void setDrivePower(float powerRightX, float powerLeftX, float powerRightY, float powerLeftY) {
        switch (driveType) {
            case TANK -> tankDrive(powerRightY, powerLeftY);
            case ARCADE -> arcadeDrive(powerLeftX, powerLeftY);
            case ZACHARIAN -> zacharianDrive(powerRightX, powerLeftX, powerRightY, powerLeftY);
            case X -> xDrive(powerRightX, powerLeftX, powerLeftY);
            case MECANUM -> mecanumDrive(powerRightX, powerLeftX, powerLeftY);
            default -> tankDrive(powerRightY, powerLeftY);
        }
        setMotorPowers();
    }

    /**Rotates to a target angle.
     * @param imuIn An IMU to find the robot's angle.
     * @param target Target angle in degrees.
     * @return True if the robot has reached the angle, false otherwise.*/
    public boolean rotateOnIMU(@NonNull BNO055IMU imuIn, float target) {
        float d = target - imuIn.getAngularOrientation().thirdAngle;//d = difference between current and target (may be backwards)
        d = (d >= 5.0f || d <= -5.0f) ? d : 0;//deadbands d to 5 degrees off
        d = (d > 30.0f) ? 1.0f : d / 50;//if d > 30 d = 1, else d / 50
        setDrivePower(d,0,  0, 0);
        return (d == 0);//if d is 0, will return true
    }

    /**Drives the robot using the dPad. Do not use if using any other drive method.
     * @param downPressed If down is pressed on the dPad.
     * @param upPressed If up is pressed on the dPad.
     * @param leftPressed If left is pressed on the dPad.
     * @param rightPressed If right is pressed on the dPad.*/
    public void dPadDrive(boolean upPressed, boolean downPressed, boolean leftPressed, boolean rightPressed) {
        if (downPressed) {
            setDrivePower(0, 0, -1, -1);
        } else if (upPressed) {
            setDrivePower(0, 0, 1, 1);
        } else if (leftPressed) {
            setDrivePower(-1, -1, 0, 0);
        } else if (rightPressed) {
            setDrivePower(1, 1, 0, 0);
        } else {
            setDrivePower(0, 0, 0, 0);
        }
    }

    /**Sets the power of each motor to the mapped motor power.*/
    private void setMotorPowers() {
        for (Map.Entry<String, Double> entry : power.entrySet()) {
            motor.get(entry.getKey()).setPower(entry.getValue());
        }
    }

    /**Updates all the pos values in the pos map.*/
    public void updatePos() {
        for (Map.Entry<String, DcMotorEx> entry : motor.entrySet()) {
            pos.replace(entry.getKey(), entry.getValue().getCurrentPosition());
        }
    }
    /**@param motorName The name of the motor to search for.
     * @return The position of the specified motor.*/
    public int getPos(String motorName) {
        updatePos();
        return pos.get(motorName);
    }
    /**@return A set of all motor names and their positions.*/
    public Set<Map.Entry<String, Integer>> getPos() {
        updatePos();
        return pos.entrySet();
    }

    /**@param motorName The name of the motor.
     * @return The vector of motion of the motor.*/
    public Vector motorVector(String motorName) { return new Vector(power.get(motorName), orientation.get(motorName).angle); }

    /**@return The vector of motion of the DriveTrain as a combination of the vectors of motion of all the motors.*/
    public Vector driveTrainVector() {
        Vector v = new Vector(0, 0);
        for (Map.Entry<String, DcMotorEx> entry : motor.entrySet()) {
            v = Geometry.add(v, motorVector(entry.getKey()));
        }
        return v;
    }
}