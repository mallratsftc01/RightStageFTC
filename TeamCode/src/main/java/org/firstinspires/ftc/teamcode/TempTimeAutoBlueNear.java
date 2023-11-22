package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.epra.*;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.util.Date;
import java.util.List;
import java.util.Random;
import android.content.Context;
import java.util.Date;
import android.os.Environment;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous
public class TempTimeAutoBlueNear extends LinearOpMode {
    private DcMotorEx northEastMotor;
    private DcMotorEx southEastMotor;
    private DcMotorEx southWestMotor;
    private DcMotorEx northWestMotor;
    private DcMotorEx rightLift;
    private DcMotorEx leftLift;
    Servo claw;
    Servo wrist;
    private Controller controller1;
    private Controller controller2;

    private TouchSensor magnet;

    private ElapsedTime runtime = new ElapsedTime();
    int step = 0;
    double stepTime = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        northEastMotor = hardwareMap.get(DcMotorEx.class, "northeastMotor");
        northWestMotor = hardwareMap.get(DcMotorEx.class, "northwestMotor");
        southEastMotor = hardwareMap.get(DcMotorEx.class, "southeastMotor");
        southWestMotor = hardwareMap.get(DcMotorEx.class, "southwestMotor");
        northWestMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        northEastMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        DriveTrain myDrive = new DriveTrain(northWestMotor, northEastMotor, southWestMotor, southEastMotor, 3, 0);

        rightLift = hardwareMap.get(DcMotorEx.class, "rightArm");
        leftLift = hardwareMap.get(DcMotorEx.class, "leftArm");
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);

        claw = hardwareMap.get(Servo.class, "clawServo");
        wrist = hardwareMap.get(Servo.class, "wristServo");

        controller1 = new Controller(gamepad1, 0.05F);
        controller2 = new Controller(gamepad2, 0.05F);

        magnet = hardwareMap.get(TouchSensor.class, "magneto");

        waitForStart();
        int saveLift = rightLift.getCurrentPosition() + 20;
        while (opModeIsActive()) {
            switch (step) {
                case 0:
                    claw.setPosition(0.75);
                    stepTime = 0.5;
                    break;
                case 1:
                   myDrive.setDrivePower(0, 0.5f, 0, 0);
                   stepTime = 0.85;
                   break;
                case 2:
                    myDrive.setDrivePower(0, 0, 0, 0);
                    rightLift.setPower(0.5);
                    stepTime = 0.7;
                    break;
                case 3:
                    rightLift.setPower(0);
                    stepTime = 1.0;
                    break;
                case 4:
                    claw.setPosition(1);
                    stepTime = 0.5;
                    break;
                case 5:
                    rightLift.setPower((rightLift.getCurrentPosition() < saveLift) ? 0 : -0.5);
                    stepTime = 0.7;
                    break;
                case 6:
                    rightLift.setPower(0);
                    myDrive.setDrivePower(0, 0, 0, 0.5f);
                    stepTime = 4;
                    break;
            }
            if (runtime.seconds() >= stepTime) {
                step++;
                runtime.reset();
            }
            if (step > 6) {break;}
            telemetry.addData("Step: ", step);
            telemetry.addData("Seconds Elapsed:", runtime.seconds());
            telemetry.update();
        }
    }
}