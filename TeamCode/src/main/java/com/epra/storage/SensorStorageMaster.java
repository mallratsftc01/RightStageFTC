package com.epra.storage;

public class SensorStorageMaster {
    public final int MOTOR_POSITIONS = 0;
     public final int MOTOR_VELOCITIES = 1;
    public final int BINARY_SENSORS = 2;
    public final int IMU_STORAGE = 3;
    public final int MOTOR_POSITION_FREQUENCY = 10;
    private int mpTracker = 0;
    public final int MOTOR_VELOCITY_FREQUENCY = 10;
    private int mvTracker = 5;
    public final int BINARY_SENSOR_FREQUENCY = 20;
    private int bsTracker = 0;
    public final int IMU_STORAGE_FREQUENCY = 50;
    private int imuTracker = 0;
    public SensorIntStorage motorPositions;
    public SensorDoubleStorage motorVelocities;
    public SensorBooleanStorage binarySensors;
    public IMUStorage imu;
    /**Stores values from sensors and coordinates their updating.
    *<p></p>
    *Queer Coded by Zachy K. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
    public SensorStorageMaster() {}
    /**Initializes the motor positions storage.*/
    public void initMotorPositions(int[] startValues) {motorPositions = new SensorIntStorage(startValues);}
    /**Initializes the motor velocities storage.*/
    public void initMotorVelocities(double[] startValues) {motorVelocities = new SensorDoubleStorage(startValues);}
    /**Initializes the binary sensors storage.*/
    public void initBinarySensors(boolean[] startValues) {binarySensors = new SensorBooleanStorage(startValues);}
    /**Initializes the imu storage.*/
    public void initIMUStorage(double yawDegrees, double pitchDegrees, double rollDegrees, double yawRadians, double pitchRadians, double rollRadians) {imu = new IMUStorage(yawDegrees,  pitchDegrees,  rollDegrees, yawRadians, pitchRadians, rollRadians);}
    /**Checks which sets are ready to be updated, returns -1 if no sets should be updated.*/
    public int checkTrackers() {
        mpTracker++; mvTracker++; bsTracker++; imuTracker++;
        if (mpTracker >= MOTOR_POSITION_FREQUENCY) {mpTracker = 0;
            return MOTOR_POSITIONS;}
        else if (mvTracker >= MOTOR_VELOCITY_FREQUENCY) {mvTracker = 0;
            return MOTOR_VELOCITIES;}
        else if (bsTracker >= BINARY_SENSOR_FREQUENCY) {bsTracker = 0;
            return BINARY_SENSORS;}
        else if (imuTracker >= IMU_STORAGE_FREQUENCY) {imuTracker = 0;
            return IMU_STORAGE;}
        else {return -1;}
    }
}
