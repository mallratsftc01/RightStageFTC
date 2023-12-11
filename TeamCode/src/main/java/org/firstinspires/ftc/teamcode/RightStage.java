package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.epra.*;
import com.qualcomm.robotcore.hardware.TouchSensor;

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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@TeleOp
public class RightStage extends LinearOpMode {
    private DcMotorEx northEastMotor;
    private DcMotorEx southEastMotor;
    private DcMotorEx southWestMotor;
    private DcMotorEx northWestMotor;
    private DcMotorEx shoulder;
    private DcMotorEx extender;
    Servo claw;
    Servo wrist;
    DrawerSlide scrollArm;
    private Controller controller1;
    private Controller controller2;

    private AprilTagProcessor aprilTag;
    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    private TouchSensor magnet;

    private IMU emu1;
    private IMU emu2;
    private IMUExpanded emu;
    private LocationServices gps;

    @Override
    public void runOpMode() throws InterruptedException {
        northEastMotor = hardwareMap.get(DcMotorEx.class, "northeastMotor");
        northWestMotor = hardwareMap.get(DcMotorEx.class, "northwestMotor");
        southEastMotor = hardwareMap.get(DcMotorEx.class, "southeastMotor");
        southWestMotor = hardwareMap.get(DcMotorEx.class, "southwestMotor");
        //southWestMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        northWestMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        northEastMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        shoulder = hardwareMap.get(DcMotorEx.class, "lift");
        extender = hardwareMap.get(DcMotorEx.class, "length");
        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoulder.setDirection(DcMotorSimple.Direction.REVERSE);

        claw = hardwareMap.get(Servo.class, "clawServo");
        wrist = hardwareMap.get(Servo.class, "wristServo");

        controller1 = new Controller (gamepad1, 0.05F);
        controller2 = new Controller (gamepad2, 0.05F);

        magnet = hardwareMap.get(TouchSensor.class, "magneto");

        scrollArm = new DrawerSlide(shoulder, extender, wrist, claw, magnet);

        initCamera();
        CameraPlus cam = new CameraPlus(aprilTag, tfod, visionPortal);

        IMU.Parameters perry = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        emu1 = hardwareMap.get(IMU.class, "imu 1");
        emu2 = hardwareMap.get(IMU.class, "imu 2");
        emu1.initialize(perry);
        emu2.initialize(perry);

        emu = new IMUExpanded(emu1, emu2);

                DriveTrain myDrive = new DriveTrain(northWestMotor, northEastMotor, southWestMotor, southEastMotor, 3, emu.avgIMU(IMUExpanded.YAW, AngleUnit.DEGREES) + 180);

        gps = new LocationServices(0, 0, northEastMotor, northWestMotor, southEastMotor, southWestMotor, 0.9f, 0.7f, controller1, cam, 12.57f, emu1);
        telemetry.addData("label: ", cam.getLabel(0));
        telemetry.addData("Num Recogs: ", cam.numRecognitions());
        telemetry.addData("ID: ", cam.getID(0));
        telemetry.addData("targetID: ", cam.getTargetID());
        telemetry.addData("left_stick_x:", controller1.left_stick_x_deadband());
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            gps.updatePositionGeneral();
            //telemetry.addData("label: ", cam.getLabel(0));
            //telemetry.addData("Num Recogs: ", cam.numRecognitions());
            telemetry.addData("ID: ", cam.getID(0));
            //arm controls
            scrollArm.moveShoulder(0.5*controller2.left_stick_y);
            scrollArm.moveExtend(0.5*controller2.right_stick_y);
            //Drive Controls
            float slow = 1 - (controller1.left_trigger_deadband() * 0.5f);
            if (controller1.buttonCase(Controller.DOWN)) {myDrive.setDrivePower(0, 0.5f * slow, 0, 0);}
            else if (controller1.buttonCase(Controller.UP)) {myDrive.setDrivePower(0, -0.5f * slow, 0, 0);}
            else if (controller1.buttonCase(Controller.LEFT)) {myDrive.setDrivePower(0, 0, 0, -0.5f * slow);}
            else if (controller1.buttonCase(Controller.RIGHT)) {myDrive.setDrivePower(0, 0, 0, 0.5f * slow);}
            else {
                if (controller1.buttonToggleSingle(Controller.A)) {
                    telemetry.addData("Target Degrees, degree distance, rPow, ", myDrive.setDrivePower(controller1.right_stick_y_deadband(), controller1.left_stick_y_deadband() * slow, controller1.right_stick_x_deadband(), controller1.left_stick_x_deadband() * slow, emu));
                    telemetry.addData("avgCurrentAngle:", (emu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + emu2.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)) / 2 + 180);}
                else {
                    myDrive.setDrivePower(controller1.right_stick_y_deadband() * slow, controller1.left_stick_y_deadband() * slow, controller1.right_stick_x_deadband() * slow, controller1.left_stick_x_deadband() * slow);
                }
            }
            telemetry.addData("2nd mode active:", controller1.buttonToggleSingle(Controller.A));
            telemetry.update();
        }
    }

    private void initCamera() {
        tfod = TfodProcessor.easyCreateWithDefaults();

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)

                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (CameraPlus.USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }

    /*private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }*/
}
