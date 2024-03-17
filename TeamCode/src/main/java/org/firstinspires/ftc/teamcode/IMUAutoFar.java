package org.firstinspires.ftc.teamcode;

import com.epra.Controller;
import com.epra.DriveTrain;
import com.epra.IMUExpanded;
import com.epra.pipelines.DualCameraElementDeterminationPipeline;
import com.epra.pipelines.ElementDeterminationPipeline;
import com.epra.storage.IMUStorage;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

@Autonomous
public class IMUAutoFar extends LinearOpMode {
    enum Path {
        RIGHT_SPIKE_NEAR_NONE,
        CENTER_SPIKE_NEAR_NONE,
        LEFT_SPIKE_NEAR_NONE,
        RIGHT_SPIKE_NEAR_BLUE,
        CENTER_SPIKE_NEAR_BLUE,
        LEFT_SPIKE_NEAR_BLUE,
        RIGHT_SPIKE_FAR_BLUE,
        CENTER_SPIKE_FAR_BLUE,
        LEFT_SPIKE_FAR_BLUE,
        DRIVE_FAR_BLUE,
        APRIL_SEARCH_BLUE,
        END_BLUE,
        RIGHT_SPIKE_NEAR_RED,
        CENTER_SPIKE_NEAR_RED,
        LEFT_SPIKE_NEAR_RED,
        RIGHT_SPIKE_FAR_RED,
        CENTER_SPIKE_FAR_RED,
        LEFT_SPIKE_FAR_RED,
        DRIVE_FAR_RED,
        APRIL_SEARCH_RED,
        BACKDROP,
        END_RED
    }

    private Path path;

    enum Location {
        NEAR,
        FAR
    }

    private Location location = Location.FAR;
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
    DrawerSlide scrollArm;
    private Controller controller1;
    private Controller controller2;

    OpenCvWebcam rightCam;
    OpenCvWebcam leftCam;
    OpenCvWebcam aprilCam;
    DualCameraElementDeterminationPipeline rightPipeline;
    DualCameraElementDeterminationPipeline leftPipeline;
    AprilTagDetectionPipeline aprilPipeline;
    ElementDeterminationPipeline.ElementPosition snapshotPos = ElementDeterminationPipeline.ElementPosition.CENTER;
    DualCameraElementDeterminationPipeline.ElementColor snapshotColor = DualCameraElementDeterminationPipeline.ElementColor.NONE;
    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    private TouchSensor magnet;

    private IMU emu1;
    private IMU emu2;
    private IMUExpanded emu;
    private YawPitchRollAngles[] orientation;
    private IMUStorage emuStorage;

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

        controller1 = new Controller(gamepad1, 0.05F);
        controller2 = new Controller(gamepad2, 0.05F);

        magnet = hardwareMap.get(TouchSensor.class, "magneto");

        scrollArm = new DrawerSlide(shoulder, extender, encoder, wrist, claw, magnet);

        //init webcam for color detection
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(
                        cameraMonitorViewId, //The container we're splitting
                        2, //The number of sub-containers to create
                        OpenCvCameraFactory.ViewportSplitMethod.HORIZONTALLY);
        telemetry.addData("l: ", viewportContainerIds.length);
        telemetry.update();
        //leftCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam Left"), viewportContainerIds[0]);
        rightCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam Right"), viewportContainerIds[1]);
        leftCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam Left"), viewportContainerIds[0]);
        //aprilCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam April"), viewportContainerIds[2]);
        rightPipeline = new DualCameraElementDeterminationPipeline(25, 195, 115, 115, 40, 40);
        leftPipeline = new DualCameraElementDeterminationPipeline(40, 210, 125, 135, 40, 40);
        rightCam.setPipeline(rightPipeline);
        rightCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                rightCam.setPipeline(rightPipeline);
                rightCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error opening the right webcam, error code ", errorCode);
                telemetry.update();
            }
        });
        leftCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                leftCam.setPipeline(leftPipeline);
                leftCam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error opening the left webcam, error code ", errorCode);
                telemetry.update();
            }
        });
        /*aprilCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened() {
                //aprilPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
                //aprilCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error opening the right webcam, error code ", errorCode);
                telemetry.update();
            }
        });*/

        //initCamera();
        //CameraPlus cam = new CameraPlus(aprilTag, tfod, visionPortal);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        emu1 = hardwareMap.get(IMU.class, "imu 1");
        emu2 = hardwareMap.get(IMU.class, "imu 2");
        emu1.initialize(new IMU.Parameters(orientationOnRobot));
        emu2.initialize(new IMU.Parameters(orientationOnRobot));
        emu = new IMUExpanded(emu1, emu2);
        orientation = new YawPitchRollAngles[2];
        for (int ii = 0; ii < orientation.length; ii++) {
            orientation[ii] = emu.getOrientation(ii);
        }

        emu1.resetYaw();
        emu2.resetYaw();

        DriveTrain myDrive = new DriveTrain(northWestMotor, northEastMotor, southWestMotor, southEastMotor, 3, emu.avgIMU(orientation, IMUExpanded.YAW, AngleUnit.DEGREES) + 180);

        //storageMaster = new SensorStorageMaster(new DcMotorEx[]{northEastMotor, northWestMotor, southEastMotor, southWestMotor, shoulder, extender}, new TouchSensor[]{magnet}, emu);

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        wrist.setPosition(-1);
        while (!isStarted() && !isStopRequested()) {
            if (leftPipeline.getPosition() != null && rightPipeline.getPosition() != null) {
                if (rightPipeline.getPosition() == DualCameraElementDeterminationPipeline.ElementPosition.LEFT) {
                    snapshotPos = ElementDeterminationPipeline.ElementPosition.LEFT;
                } else if (leftPipeline.getPosition() == DualCameraElementDeterminationPipeline.ElementPosition.RIGHT) {
                    snapshotPos = ElementDeterminationPipeline.ElementPosition.RIGHT;
                } else if (rightPipeline.getPosition() == DualCameraElementDeterminationPipeline.ElementPosition.RIGHT || leftPipeline.getPosition() == DualCameraElementDeterminationPipeline.ElementPosition.LEFT) {
                    snapshotPos = ElementDeterminationPipeline.ElementPosition.CENTER;
                }

                if (leftPipeline.getColor() == DualCameraElementDeterminationPipeline.ElementColor.RED || rightPipeline.getColor() == DualCameraElementDeterminationPipeline.ElementColor.RED) {
                    snapshotColor = DualCameraElementDeterminationPipeline.ElementColor.RED;
                } else if (leftPipeline.getColor() == DualCameraElementDeterminationPipeline.ElementColor.BLUE || rightPipeline.getColor() == DualCameraElementDeterminationPipeline.ElementColor.BLUE) {
                    snapshotColor = DualCameraElementDeterminationPipeline.ElementColor.BLUE;
                } else {
                    snapshotColor = DualCameraElementDeterminationPipeline.ElementColor.NONE;
                }
                telemetry.addData("pos: ", snapshotPos);
                telemetry.addData("color: ", snapshotColor);
                telemetry.update();
            }
        }
        wrist.setPosition(1);
        String p = snapshotPos.toString() + "_SPIKE_" + location.toString() + "_" + snapshotColor.toString();
        path = Path.valueOf(p);
        startTime = System.currentTimeMillis();
        long saveTime = startTime;
        int step = 0;
        while (opModeIsActive()) {
            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }
            for (int ii = 0; ii < orientation.length; ii++) {
                orientation[ii] = emu.getOrientation(ii);
            }

            if (true && (path == Path.APRIL_SEARCH_BLUE || path == Path.APRIL_SEARCH_RED)) {
                path = Path.BACKDROP;
            }

            if (path == Path.CENTER_SPIKE_NEAR_RED) {
                switch (step) {
                    case 0:
                        claw.setPower(0);
                        myDrive.setDrivePower(0, -0.5f, 0, 0, emu, orientation);
                        if (System.currentTimeMillis() - saveTime > 1500) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 1:
                        claw.setPower(-1);
                        myDrive.setDrivePower(0, 0, 0, 0);
                        if (System.currentTimeMillis() - saveTime > 100) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 2:
                        claw.setPower(-1);
                        if (System.currentTimeMillis() - saveTime > 125) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 3:
                        wrist.setPosition(0);
                        claw.setPower(1);
                        myDrive.setDrivePower(0, 0.4f, 0, 0, emu, orientation);
                        if (System.currentTimeMillis() - saveTime > 400) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 4:
                        claw.setPower(0);
                        myDrive.setDrivePower(0, 0, 0.35f, 0);
                        if (Math.abs(emu.trueDistIMU(orientation, IMUExpanded.YAW, AngleUnit.DEGREES, -90)) < 5) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 5:
                        myDrive.setDrivePower(0, 0, 0, 0);
                        saveTime = System.currentTimeMillis();
                        path = Path.APRIL_SEARCH_RED;
                        step = 0;
                        break;
                }
            }

            if (path == Path.LEFT_SPIKE_NEAR_RED) {
                switch (step) {
                    case 0:
                        myDrive.setDrivePower(0, -0.5f, 0, 0, emu, orientation);
                        if (System.currentTimeMillis() - saveTime > 1250) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 1:
                        claw.setPower(0);
                        myDrive.setDrivePower(0, 0, -0.35f, 0);
                        if (Math.abs(emu.trueDistIMU(orientation, IMUExpanded.YAW, AngleUnit.DEGREES, 65)) < 5) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 2:
                        myDrive.setDrivePower(0, 0, 0, 0);
                        claw.setPower(-1);
                        if (System.currentTimeMillis() - saveTime > 250) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 3:
                        wrist.setPosition(-1);
                        claw.setPower(1);
                        myDrive.setDrivePower(0, 0.3f, 0, 0);
                        if (System.currentTimeMillis() - saveTime > 300) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 4:
                        claw.setPower(0);
                        myDrive.setDrivePower(0, 0, 0.35f, 0);
                        if (Math.abs(emu.trueDistIMU(orientation, IMUExpanded.YAW, AngleUnit.DEGREES, -90)) < 5) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 5:
                        myDrive.setDrivePower(0, 0, 0, 0);
                        saveTime = System.currentTimeMillis();
                        path = Path.APRIL_SEARCH_RED;
                        step = 0;
                        break;
                }
            }

            if (path == Path.RIGHT_SPIKE_NEAR_RED) {
                switch (step) {
                    case 0:
                        myDrive.setDrivePower(0, -0.5f, 0, 0, emu, orientation);
                        if (System.currentTimeMillis() - saveTime > 1250) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 1:
                        claw.setPower(0);
                        myDrive.setDrivePower(0, 0, 0.35f, 0);
                        if (Math.abs(emu.trueDistIMU(orientation, IMUExpanded.YAW, AngleUnit.DEGREES, -35)) < 5) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 2:
                        myDrive.setDrivePower(0, 0, 0, 0);
                        claw.setPower(-1);
                        if (System.currentTimeMillis() - saveTime > 250) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 3:
                        wrist.setPosition(-1);
                        claw.setPower(1);
                        myDrive.setDrivePower(0, 0.3f, 0, 0);
                        if (System.currentTimeMillis() - saveTime > 400) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 4:
                        claw.setPower(0);
                        myDrive.setDrivePower(0, 0, 0.35f, 0);
                        if (Math.abs(emu.trueDistIMU(orientation, IMUExpanded.YAW, AngleUnit.DEGREES, -90)) < 5) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 5:
                        myDrive.setDrivePower(0, 0, 0, 0);
                        saveTime = System.currentTimeMillis();
                        path = Path.APRIL_SEARCH_RED;
                        step = 0;
                        break;
                }
            }

            if (path == Path.LEFT_SPIKE_FAR_RED) {
                switch (step) {
                    case 0:
                        myDrive.setDrivePower(0, -0.5f, 0, 0, emu, orientation);
                        if (System.currentTimeMillis() - saveTime > 1250) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 1:
                        claw.setPower(0);
                        myDrive.setDrivePower(0, 0, -0.35f, 0);
                        if (Math.abs(emu.trueDistIMU(orientation, IMUExpanded.YAW, AngleUnit.DEGREES, 60)) < 5) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 2:
                        myDrive.setDrivePower(0, 0, 0, 0);
                        claw.setPower(-1);
                        if (System.currentTimeMillis() - saveTime > 225) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 3:
                        wrist.setPosition(-1);
                        claw.setPower(1);
                        myDrive.setDrivePower(0, 0.4f, 0, 0);
                        if (System.currentTimeMillis() - saveTime > 300) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 4:
                        claw.setPower(0);
                        myDrive.setDrivePower(0, 0, 0.35f, 0);
                        if (Math.abs(emu.trueDistIMU(orientation, IMUExpanded.YAW, AngleUnit.DEGREES, 20)) < 5) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 5:
                        myDrive.setDrivePower(0, -0.5f, 0, 0, emu, orientation);
                        if (System.currentTimeMillis() - saveTime > 1700) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 6:
                        myDrive.setDrivePower(0, 0, 0.35f, 0);
                        if (Math.abs(emu.trueDistIMU(orientation, IMUExpanded.YAW, AngleUnit.DEGREES, -90)) < 5) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 7:
                        myDrive.setDrivePower(0, 0, 0, 0);
                        saveTime = System.currentTimeMillis();
                        path = Path.DRIVE_FAR_RED;
                        step = 0;
                        break;
                }
            }

            if (path == Path.RIGHT_SPIKE_FAR_RED) {
                switch (step) {
                    case 0:
                        myDrive.setDrivePower(0, -0.5f, 0, 0, emu, orientation);
                        if (System.currentTimeMillis() - saveTime > 1150) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 1:
                        claw.setPower(0);
                        myDrive.setDrivePower(0, 0, 0.35f, 0);
                        if (Math.abs(emu.trueDistIMU(orientation, IMUExpanded.YAW, AngleUnit.DEGREES, -45)) < 5) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 2:
                        myDrive.setDrivePower(0, 0, 0, 0);
                        claw.setPower(-1);
                        if (System.currentTimeMillis() - saveTime > 250) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 3:
                        wrist.setPosition(-1);
                        claw.setPower(1);
                        myDrive.setDrivePower(0, 0.2f, 0, 0);
                        if (System.currentTimeMillis() - saveTime > 300) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 4:
                        claw.setPower(0);
                        myDrive.setDrivePower(0, 0, -0.35f, 0);
                        if (Math.abs(emu.trueDistIMU(orientation, IMUExpanded.YAW, AngleUnit.DEGREES, 20)) < 5) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 5:
                        myDrive.setDrivePower(0, -0.5f, 0, 0, emu, orientation);
                        if (System.currentTimeMillis() - saveTime > 1550) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 6:
                        myDrive.setDrivePower(0, 0, 0.35f, 0);
                        if (Math.abs(emu.trueDistIMU(orientation, IMUExpanded.YAW, AngleUnit.DEGREES, -90)) < 5) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 7:
                        myDrive.setDrivePower(0, 0, 0, 0);
                        saveTime = System.currentTimeMillis();
                        path = Path.DRIVE_FAR_RED;
                        step = 0;
                        break;
                }
            }

            if (path == Path.CENTER_SPIKE_FAR_RED) {
                switch (step) {
                    case 0:
                        myDrive.setDrivePower(0, -0.5f, 0, 0, emu, orientation);
                        if (System.currentTimeMillis() - saveTime > 3150) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 1:
                        claw.setPower(0);
                        myDrive.setDrivePower(0, 0, -0.35f, 0);
                        if (Math.abs(emu.trueDistIMU(orientation, IMUExpanded.YAW, AngleUnit.DEGREES, 180)) < 5) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 2:
                        myDrive.setDrivePower(0, 0, 0, 0);
                        claw.setPower(-1);
                        if (System.currentTimeMillis() - saveTime > 225) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 3:
                        wrist.setPosition(-1);
                        claw.setPower(1);
                        myDrive.setDrivePower(0, 0, 0, 0);
                        if (System.currentTimeMillis() - saveTime > 300) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 4:
                        claw.setPower(0);
                        myDrive.setDrivePower(0, 0, -0.35f, 0);
                        if (Math.abs(emu.trueDistIMU(orientation, IMUExpanded.YAW, AngleUnit.DEGREES, -90)) < 5) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 5:
                        myDrive.setDrivePower(0, 0, 0, 0);
                        saveTime = System.currentTimeMillis();
                        path = Path.DRIVE_FAR_RED;
                        step = 0;
                        break;
                }
            }

            if (path == Path.DRIVE_FAR_RED) {
                switch (step) {
                    case 0:
                        emu1.resetYaw();
                        emu2.resetYaw();
                        myDrive.setDrivePower(0, -0.5f, 0, 0, emu, orientation);
                        if (System.currentTimeMillis() - saveTime > 3200) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 1:
                        myDrive.setDrivePower(0, 0, 0, 0.5f, emu, orientation);
                        if (System.currentTimeMillis() - saveTime > 1700) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 2:
                        myDrive.setDrivePower(0, 0, 0, 0);
                        saveTime = System.currentTimeMillis();
                        path = Path.APRIL_SEARCH_RED;
                        step = 0;
                        break;
                }
            }

            if (path == Path.BACKDROP) {
                switch (step) {
                    case 0:
                        emu1.resetYaw();
                        emu2.resetYaw();
                        shoulder.setPower(-0.5);
                        if (scrollArm.getShoulderDegree() > 0) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 1:
                        shoulder.setPower(0);
                        myDrive.setDrivePower(0, -0.5f, 0, 0, emu, orientation);
                        if (System.currentTimeMillis() - saveTime > 1300) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 2:
                        claw.setPower(-1);
                        wrist.setPosition(1);
                        myDrive.setDrivePower(0, 0, 0, 0, emu, orientation);
                        if (System.currentTimeMillis() - saveTime > 700) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 3:
                        claw.setPower(0);
                        myDrive.setDrivePower(0, 0, 0, 0);
                        saveTime = System.currentTimeMillis();
                        path = Path.valueOf("END_" + snapshotColor.toString());
                        step = 0;
                        break;
                }
            }

            if (path == Path.END_RED) {
                switch (step) {
                    case 0:
                        myDrive.setDrivePower(0, 0, 0, 0.5f, emu, orientation);
                        if (System.currentTimeMillis() - saveTime > 2300) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 1:
                        myDrive.setDrivePower(0, -0.5f, 0, 0, emu, orientation);
                        if (System.currentTimeMillis() - saveTime > 500) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 2:
                        claw.setPower(0);
                        myDrive.setDrivePower(0, 0, 0, 0);
                        saveTime = System.currentTimeMillis();
                        break;
                }
            }

            //BLUE

            if (path == Path.CENTER_SPIKE_NEAR_BLUE) {
                switch (step) {
                    case 0:
                        claw.setPower(0);
                        myDrive.setDrivePower(0, -0.5f, 0, 0, emu, orientation);
                        if (System.currentTimeMillis() - saveTime > 1500) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 1:
                        claw.setPower(-1);
                        myDrive.setDrivePower(0, 0, 0, 0);
                        if (System.currentTimeMillis() - saveTime > 100) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 2:
                        claw.setPower(-1);
                        if (System.currentTimeMillis() - saveTime > 125) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 3:
                        wrist.setPosition(0);
                        claw.setPower(1);
                        myDrive.setDrivePower(0, 0.4f, 0, 0, emu, orientation);
                        if (System.currentTimeMillis() - saveTime > 400) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 4:
                        claw.setPower(0);
                        myDrive.setDrivePower(0, 0, -0.35f, 0);
                        if (Math.abs(emu.trueDistIMU(orientation, IMUExpanded.YAW, AngleUnit.DEGREES, 90)) < 5) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 5:
                        myDrive.setDrivePower(0, 0, 0, 0);
                        saveTime = System.currentTimeMillis();
                        path = Path.APRIL_SEARCH_RED;
                        step = 0;
                        break;
                }
            }

            if (path == Path.RIGHT_SPIKE_NEAR_BLUE) {
                switch (step) {
                    case 0:
                        myDrive.setDrivePower(0, -0.5f, 0, 0, emu, orientation);
                        if (System.currentTimeMillis() - saveTime > 1250) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 1:
                        claw.setPower(0);
                        myDrive.setDrivePower(0, 0, 0.35f, 0);
                        if (Math.abs(emu.trueDistIMU(orientation, IMUExpanded.YAW, AngleUnit.DEGREES, -45)) < 5) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 2:
                        myDrive.setDrivePower(0, 0, 0, 0);
                        claw.setPower(-1);
                        if (System.currentTimeMillis() - saveTime > 250) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 3:
                        wrist.setPosition(-1);
                        claw.setPower(1);
                        myDrive.setDrivePower(0, 0.3f, 0, 0);
                        if (System.currentTimeMillis() - saveTime > 300) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 4:
                        claw.setPower(0);
                        myDrive.setDrivePower(0, 0, -0.35f, 0);
                        if (Math.abs(emu.trueDistIMU(orientation, IMUExpanded.YAW, AngleUnit.DEGREES, 90)) < 5) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 5:
                        myDrive.setDrivePower(0, 0, 0, 0);
                        saveTime = System.currentTimeMillis();
                        path = Path.APRIL_SEARCH_RED;
                        step = 0;
                        break;
                }
            }

            if (path == Path.LEFT_SPIKE_NEAR_BLUE) {
                switch (step) {
                    case 0:
                        myDrive.setDrivePower(0, -0.5f, 0, 0, emu, orientation);
                        if (System.currentTimeMillis() - saveTime > 1500) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 1:
                        claw.setPower(0);
                        myDrive.setDrivePower(0, 0, -0.35f, 0);
                        if (Math.abs(emu.trueDistIMU(orientation, IMUExpanded.YAW, AngleUnit.DEGREES, 50)) < 5) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 2:
                        myDrive.setDrivePower(0, 0, 0, 0);
                        claw.setPower(-1);
                        if (System.currentTimeMillis() - saveTime > 250) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 3:
                        wrist.setPosition(-1);
                        claw.setPower(1);
                        myDrive.setDrivePower(0, 0.3f, 0, 0);
                        if (System.currentTimeMillis() - saveTime > 400) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 4:
                        claw.setPower(0);
                        myDrive.setDrivePower(0, 0, -0.35f, 0);
                        if (Math.abs(emu.trueDistIMU(orientation, IMUExpanded.YAW, AngleUnit.DEGREES, 90)) < 5) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 5:
                        myDrive.setDrivePower(0, 0, 0, 0);
                        saveTime = System.currentTimeMillis();
                        path = Path.APRIL_SEARCH_RED;
                        step = 0;
                        break;
                }
            }

            if (path == Path.RIGHT_SPIKE_FAR_BLUE) {
                switch (step) {
                    case 0:
                        myDrive.setDrivePower(0, -0.5f, 0, 0, emu, orientation);
                        if (System.currentTimeMillis() - saveTime > 1250) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 1:
                        claw.setPower(0);
                        myDrive.setDrivePower(0, 0, 0.35f, 0);
                        if (Math.abs(emu.trueDistIMU(orientation, IMUExpanded.YAW, AngleUnit.DEGREES, -40)) < 5) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 2:
                        myDrive.setDrivePower(0, 0, 0, 0);
                        claw.setPower(-1);
                        if (System.currentTimeMillis() - saveTime > 245) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 3:
                        wrist.setPosition(-1);
                        claw.setPower(1);
                        myDrive.setDrivePower(0, 0.4f, 0, 0);
                        if (System.currentTimeMillis() - saveTime > 300) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 4:
                        claw.setPower(0);
                        myDrive.setDrivePower(0, 0, -0.35f, 0);
                        if (Math.abs(emu.trueDistIMU(orientation, IMUExpanded.YAW, AngleUnit.DEGREES, -20)) < 5) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 5:
                        myDrive.setDrivePower(0, -0.5f, 0, 0, emu, orientation);
                        if (System.currentTimeMillis() - saveTime > 1700) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 6:
                        myDrive.setDrivePower(0, 0, -0.35f, 0);
                        if (Math.abs(emu.trueDistIMU(orientation, IMUExpanded.YAW, AngleUnit.DEGREES, 90)) < 5) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 7:
                        myDrive.setDrivePower(0, 0, 0, 0);
                        saveTime = System.currentTimeMillis();
                        path = Path.DRIVE_FAR_BLUE;
                        step = 0;
                        break;
                }
            }

            if (path == Path.LEFT_SPIKE_FAR_BLUE) {
                switch (step) {
                    case 0:
                        myDrive.setDrivePower(0, -0.5f, 0, 0, emu, orientation);
                        if (System.currentTimeMillis() - saveTime > 1150) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 1:
                        claw.setPower(0);
                        myDrive.setDrivePower(0, 0, -0.35f, 0);
                        if (Math.abs(emu.trueDistIMU(orientation, IMUExpanded.YAW, AngleUnit.DEGREES, 70)) < 5) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 2:
                        myDrive.setDrivePower(0, 0, 0, 0);
                        claw.setPower(-1);
                        if (System.currentTimeMillis() - saveTime > 250) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 3:
                        wrist.setPosition(-1);
                        claw.setPower(1);
                        myDrive.setDrivePower(0, 0.2f, 0, 0);
                        if (System.currentTimeMillis() - saveTime > 300) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 4:
                        claw.setPower(0);
                        myDrive.setDrivePower(0, 0, 0.35f, 0);
                        if (Math.abs(emu.trueDistIMU(orientation, IMUExpanded.YAW, AngleUnit.DEGREES, -20)) < 5) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 5:
                        myDrive.setDrivePower(0, -0.5f, 0, 0, emu, orientation);
                        if (System.currentTimeMillis() - saveTime > 1550) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 6:
                        myDrive.setDrivePower(0, 0, -0.35f, 0);
                        if (Math.abs(emu.trueDistIMU(orientation, IMUExpanded.YAW, AngleUnit.DEGREES, 90)) < 5) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 7:
                        myDrive.setDrivePower(0, 0, 0, 0);
                        saveTime = System.currentTimeMillis();
                        path = Path.DRIVE_FAR_BLUE;
                        step = 0;
                        break;
                }
            }

            if (path == Path.CENTER_SPIKE_FAR_BLUE) {
                switch (step) {
                    case 0:
                        myDrive.setDrivePower(0, -0.5f, 0, 0, emu, orientation);
                        if (System.currentTimeMillis() - saveTime > 2800) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 1:
                        claw.setPower(0);
                        myDrive.setDrivePower(0, 0, 0.35f, 0);
                        if (Math.abs(emu.trueDistIMU(orientation, IMUExpanded.YAW, AngleUnit.DEGREES, -180)) < 5) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 2:
                        myDrive.setDrivePower(0, 0, 0, 0.5f);
                        claw.setPower(-1);
                        if (System.currentTimeMillis() - saveTime > 225) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 3:
                        myDrive.setDrivePower(0, 0, 0, 0);
                        claw.setPower(-1);
                        if (System.currentTimeMillis() - saveTime > 225) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 4:
                        wrist.setPosition(-1);
                        claw.setPower(1);
                        myDrive.setDrivePower(0, 0.3f, 0, 0);
                        if (System.currentTimeMillis() - saveTime > 300) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 5:
                        claw.setPower(0);
                        myDrive.setDrivePower(0, 0, 0.35f, 0);
                        if (Math.abs(emu.trueDistIMU(orientation, IMUExpanded.YAW, AngleUnit.DEGREES, 90)) < 5) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 6:
                        myDrive.setDrivePower(0, 0, 0, 0);
                        saveTime = System.currentTimeMillis();
                        path = Path.DRIVE_FAR_BLUE;
                        step = 0;
                        break;
                }
            }

            if (path == Path.DRIVE_FAR_BLUE) {
                switch (step) {
                    case 0:
                        emu1.resetYaw();
                        emu2.resetYaw();
                        myDrive.setDrivePower(0, -0.5f, 0, 0, emu, orientation);
                        if (System.currentTimeMillis() - saveTime > 2800) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 1:
                        myDrive.setDrivePower(0, 0, 0, -0.5f, emu, orientation);
                        if (System.currentTimeMillis() - saveTime > 2100) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 2:
                        myDrive.setDrivePower(0, 0, 0, 0);
                        saveTime = System.currentTimeMillis();
                        path = Path.APRIL_SEARCH_BLUE;
                        step = 0;
                        break;
                }
            }

            if (path == Path.END_BLUE) {
                switch (step) {
                    case 0:
                        myDrive.setDrivePower(0, 0, 0, -0.5f, emu, orientation);
                        if (System.currentTimeMillis() - saveTime > 2300) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 1:
                        myDrive.setDrivePower(0, -0.5f, 0, 0, emu, orientation);
                        if (System.currentTimeMillis() - saveTime > 500) {
                            step++;
                            saveTime = System.currentTimeMillis();
                        }
                        break;
                    case 2:
                        claw.setPower(0);
                        myDrive.setDrivePower(0, 0, 0, 0);
                        saveTime = System.currentTimeMillis();
                        break;
                }
            }


            telemetry.addData("Yaw: ", emu.avgIMU(orientation, IMUExpanded.YAW, AngleUnit.DEGREES));
            telemetry.addData("Dist: ", emu.trueDistIMU(orientation, IMUExpanded.YAW, AngleUnit.DEGREES, -90));
            telemetry.addData("Path: ", path.toString());
            telemetry.addData("Step: ", step);
            telemetry.update();
        }
    }
}


