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

    OpenCvWebcam rightCam;
    OpenCvWebcam leftCam;
    OpenCvWebcam aprilCam;
    UniversalColorDeterminationPipeline rightPipeline;
    UniversalColorDeterminationPipeline leftPipeline;
    AprilTagDetectionPipeline aprilPipeline;
    int elementPos = 2;
    UniversalColorDeterminationPipeline.ElementColor snapshotColor = UniversalColorDeterminationPipeline.ElementColor.BLUE;
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
        plane = hardwareMap.get(CRServo.class, "drone");

        controller1 = new Controller (gamepad1, 0.05F);
        controller2 = new Controller (gamepad2, 0.05F);

        magnet = hardwareMap.get(TouchSensor.class, "magneto");

        scrollArm = new DrawerSlide(shoulder, extender, encoder, wrist, claw, magnet);

        //init webcam for color detection
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(
                        cameraMonitorViewId, //The container we're splitting
                        3, //The number of sub-containers to create
                        OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY);
        rightCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), viewportContainerIds[0]);
        leftCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), viewportContainerIds[1]);
        aprilCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 3"), viewportContainerIds[2]);
        rightPipeline = new UniversalColorDeterminationPipeline(new int[] {80, 200}, new int[] {100, 120}, 20, 20, CR, CB, TOLERANCE);
        leftPipeline = new UniversalColorDeterminationPipeline(new int[] {80, 200}, new int[] {100, 120}, 20, 20, CR, CB, TOLERANCE);
        aprilPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        rightCam.setPipeline(rightPipeline);
        leftCam.setPipeline(leftPipeline);
        leftCam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        rightCam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        aprilCam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        leftCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened() {
                leftCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error opening the left webcam, error code ", errorCode);
                telemetry.update();
            }
        });
        rightCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened() {
                rightCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error opening the right webcam, error code ", errorCode);
                telemetry.update();
            }
        });
        aprilCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened() {
                aprilCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error opening the right webcam, error code ", errorCode);
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
                claw.setPower((controller2.analogDeadband(Controller.Button.LEFT_TRIGGER) + controller1.buttonSingleInt(Controller.Button.BUMPER_LEFT)) - (controller2.analogDeadband(Controller.Button.RIGHT_TRIGGER)) + controller1.buttonSingleInt(Controller.Button.BUMPER_RIGHT));
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
}

