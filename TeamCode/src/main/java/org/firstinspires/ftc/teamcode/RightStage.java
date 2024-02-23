package org.firstinspires.ftc.teamcode;

import com.epra.*;
import com.epra.storage.*;
import com.epra.pipelines.*;
import com.epra.location.*;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.epra.*;
import com.qualcomm.robotcore.hardware.TouchSensor;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp
public class RightStage extends LinearOpMode {
    private static final int CR = 240;
    private static final int CB = 0;
    private static final int TOLERANCE = 40;
    private DcMotorEx northEastMotor;
    private DcMotorEx southEastMotor;
    private DcMotorEx southWestMotor;
    private DcMotorEx northWestMotor;
    private DcMotorEx shoulder;
    private DcMotorEx extender;
    private DcMotorEx encoder;
    CRServo claw;
    Servo wrist;
    int clawFlag = 0;
    CRServo plane;
    DrawerSlide scrollArm;
    private Controller controller1;
    private Controller controller2;

    private AprilTagProcessor aprilTag;
    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    OpenCvWebcam webcam;
    UniversalColorDeterminationPipeline pipeline;
    int elementPos = 2;
    UniversalColorDeterminationPipeline.ElementColor snapshotColor = UniversalColorDeterminationPipeline.ElementColor.BLUE;

    private TouchSensor magnet;

    private IMU emu1;
    private IMU emu2;
    private IMUExpanded emu;
    private YawPitchRollAngles[] orientation;
    private IMUStorage emuStorage;
    private LocationServices gps;

    //SensorStorageMaster storageMaster;
    List<LynxModule> allHubs;

    long startTime;
    long timesRun;

    @Override
    public void runOpMode() throws InterruptedException {
        northEastMotor = hardwareMap.get(DcMotorEx.class, "northeastMotor");
        northWestMotor = hardwareMap.get(DcMotorEx.class, "northwestMotor");
        southEastMotor = hardwareMap.get(DcMotorEx.class, "southeastMotor");
        southWestMotor = hardwareMap.get(DcMotorEx.class, "southwestMotor");
        northWestMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        northEastMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        shoulder = hardwareMap.get(DcMotorEx.class, "lift");
        extender = hardwareMap.get(DcMotorEx.class, "length");
        encoder = hardwareMap.get(DcMotorEx.class, "measure");
        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        encoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shoulder.setDirection(DcMotorSimple.Direction.REVERSE);

        claw = hardwareMap.get(CRServo.class, "clawServo");
        wrist = hardwareMap.get(Servo.class, "wristServo");
        plane = hardwareMap.get(CRServo.class, "drone");

        controller1 = new Controller (gamepad1, 0.05F);
        controller2 = new Controller (gamepad2, 0.05F);

        magnet = hardwareMap.get(TouchSensor.class, "magneto");

        scrollArm = new DrawerSlide(shoulder, extender, encoder, wrist, claw, magnet);

        //init webcam for color detection
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new UniversalColorDeterminationPipeline(new int[] {80, 200}, new int[] {100, 120}, 20, 20, CR, CB, TOLERANCE);
        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error opening the webcam, error code ", errorCode);
                telemetry.update();
            }
        });

        //initCamera();
        //CameraPlus cam = new CameraPlus(aprilTag, tfod, visionPortal);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        emu1 = hardwareMap.get(IMU.class, "imu 1");
        emu2 = hardwareMap.get(IMU.class, "imu 2");
        emu1.initialize(new IMU.Parameters(orientationOnRobot));
        emu2.initialize(new IMU.Parameters(orientationOnRobot));
        emu = new IMUExpanded(emu1, emu2);
        orientation = new YawPitchRollAngles[2];
        for (int ii = 0; ii < orientation.length; ii++) {orientation[ii] = emu.getOrientation(ii);}

        DriveTrain myDrive = new DriveTrain(northWestMotor, northEastMotor, southWestMotor, southEastMotor, 3, emu.avgIMU(orientation, IMUExpanded.YAW, AngleUnit.DEGREES) + 180);

        //storageMaster = new SensorStorageMaster(new DcMotorEx[]{northEastMotor, northWestMotor, southEastMotor, southWestMotor, shoulder, extender}, new TouchSensor[]{magnet}, emu);

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        waitForStart();
        startTime = System.currentTimeMillis() - 1000;
        while (opModeIsActive()) {
            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }
            //storageMaster.update();
            for (int ii = 0; ii < orientation.length; ii++) {orientation[ii] = emu.getOrientation(ii);}
            telemetry.addData("Yaw: ", emu.avgIMU(orientation, IMUExpanded.YAW, AngleUnit.DEGREES));
            telemetry.addData("Pitch: ", emu.avgIMU(orientation, IMUExpanded.PITCH, AngleUnit.DEGREES));
            telemetry.addData("Roll: ", emu.avgIMU(orientation, IMUExpanded.ROLL, AngleUnit.DEGREES));
            //gps.updatePositionGeneral();
            //telemetry.addData("label: ", cam.getLabel(0));
            //telemetry.addData("Num Recogs: ", cam.numRecognitions());
            //telemetry.addData("ID: ", cam.getID(0));
            //telemetry.addData("Yaw: ", storageMaster.imuStorage.avgIMU(IMUExpanded.YAW, AngleUnit.DEGREES));
            telemetry.addData("Time Since Start", System.currentTimeMillis() - startTime);
            telemetry.addData("Times Looped", ++timesRun);
            telemetry.addData("Loops per Second", timesRun / ((System.currentTimeMillis() - startTime)/1000.0));
            //arm controls
            double shoulderPow = 0.0;
            /*switch (controller2.buttonCounterSingle(Controller.Button.B, 2)) {
                case 0:
                    shoulderPow = scrollArm.targetShoulderDegree(120.0);
                    break;
                case 1:
                    shoulderPow = scrollArm.targetShoulderDegree(185.0);
                    break;
            }
            shoulderPow *= (controller2.buttonCase(Controller.Button.X)) ? 0.0 : -1.0;*/
            telemetry.addData("pow: ", shoulderPow);
            telemetry.addData("degree: ", scrollArm.getShoulderDegree());
            shoulderPow = controller2.analogDeadband(Controller.Button.RIGHT_STICK_Y) * 0.5;
            scrollArm.moveShoulder(shoulderPow);
            //scrollArm.moveShoulder(controller2.analogDeadband(Controller.Button.LEFT_STICK_Y) * 0.5);
            //scrollArm.moveExtend(0.5 * controller2.analogDeadband(Controller.Button.RIGHT_STICK_Y));

            if (controller2.buttonCase(Controller.Button.BUMPER_LEFT)) {
                if (clawFlag < 16) {
                    clawFlag++;
                    claw.setPower(-1);
                } else {claw.setPower(0);}
            } else {
                clawFlag = 0;
                claw.setPower(controller2.analogDeadband(Controller.Button.LEFT_TRIGGER) - controller2.analogDeadband(Controller.Button.RIGHT_TRIGGER));
            }
            wrist.setPosition((controller2.buttonToggleSingle(Controller.Button.BUMPER_RIGHT)) ? 1 : -1);
            /*if (myDrive.getAverageVelocity() < 300 || controller2.buttonToggleSingle(Controller.Button.A)) {);
            } else {wrist.setPosition(-1);}*/

            //scrollArm.moveExtendMagnet(controller2.analogDeadband(Controller.Button.RIGHT_STICK_Y), -0.5*controller2.analogDeadband(Controller.Button.RIGHT_STICK_Y));
            telemetry.addData("Extend pos: ", extender.getCurrentPosition());
            telemetry.addData("Shoulder pos: ", scrollArm.getShoulderDist());
            telemetry.addData("Shoulder velo: ", shoulder.getVelocity());

            plane.setPower((controller1.buttonCase(Controller.Button.Y) && controller2.buttonCase(Controller.Button.Y)) ? -1.0 : 0);
            telemetry.addData("Plane connection: ", plane.getConnectionInfo());
            telemetry.addData("Ys pressed: ", controller1.buttonCase(Controller.Button.Y) && controller2.buttonCase(Controller.Button.Y));
            telemetry.addData("plane: ", plane.getPower());

            //Drive Control
            float slow = 1 - (controller1.analogDeadband(Controller.Button.LEFT_TRIGGER) * 0.5f);
            //dpad drive
            if (controller1.buttonCase(Controller.Button.DOWN)) {myDrive.setDrivePower(0, 0.5f * slow, 0, 0);}
            else if (controller1.buttonCase(Controller.Button.UP)) {myDrive.setDrivePower(0, -0.5f * slow, 0, 0);}
            else if (controller1.buttonCase(Controller.Button.LEFT)) {myDrive.setDrivePower(0, 0, 0, -0.5f * slow);}
            else if (controller1.buttonCase(Controller.Button.RIGHT)) {myDrive.setDrivePower(0, 0, 0, 0.5f * slow);}
            //default to normal drive
            else {myDrive.setDrivePower(controller1.analogDeadband(Controller.Button.RIGHT_STICK_Y) * slow, controller1.analogDeadband(Controller.Button.LEFT_STICK_Y) * slow, controller1.analogDeadband(Controller.Button.RIGHT_STICK_X) * slow, controller1.analogDeadband(Controller.Button.LEFT_STICK_X) * slow);}
            telemetry.update();
        }
    }

    private void initCamera() {

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

