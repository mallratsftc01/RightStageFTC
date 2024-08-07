package org.firstinspires.ftc.teamcode;

import com.epra.DriveTrain;
import com.epra.location.AprilTagLocator;
import com.epra.pipelines.ElementDeterminationPipeline;
import com.epra.pipelines.UniversalColorDeterminationPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
@Disabled
public class AutoNearBlue extends LinearOpMode {
    private static final int CR = 240;
    private static final int CB = 0;
    private static final int TOLERANCE = 40;
    private static final boolean NEAR = true;
    private DcMotorEx northEastMotor;
    private DcMotorEx southEastMotor;
    private DcMotorEx southWestMotor;
    private DcMotorEx northWestMotor;
    private DcMotorEx extend;
    private DcMotorEx shoulder;
    private DcMotorEx encoder;
    CRServo claw;
    Servo wrist;
    private TouchSensor magnet;
    private DrawerSlide scrollArm;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private AprilTagLocator locator;
    int targetTag = 1;

    private OpenCvWebcam webcam;
    private ElementDeterminationPipeline ePipeline;
    UniversalColorDeterminationPipeline pipeline;
    int elementPos = 2;
    UniversalColorDeterminationPipeline.ElementColor snapshotColor = UniversalColorDeterminationPipeline.ElementColor.RED;
    private enum StartingLocation {
        RED_NEAR,
        RED_FAR,
        BLUE_NEAR,
        BLUE_FAR
    }
    public enum Step {
        MOD_B_1 (0, 1500, 1000000000),
        MOD_B_2 (1, 3000, 10000000),
        MOD_B_3 (2, 100, 10000000),
        MOD_B_4 (3, 400, 10000000),
        MOD_B_5 (4, 30000, 1000000000),
        MOD_B_6 (5, 1750, 10000000),
        MOD_B_END(6, 500, 1000000000),
        MOD_C_1 (7, 1100, 10000000),
        MOD_C_2 (8, 2100, 10000000),
        MOD_C_3 (9, 900, 10000000),
        MOD_D (10, 2400, 10000000),
        MOD_C_END (11, 100, 10000000),
        MOD_E_1 (12, 500, 10000000),
        //MOD_E_2 (12, 10000, 10000000),
        MOD_E_LEFT (13, 1800, 10000000),
        MOD_E_CENTER (13, 1900, 10000000),
        MOD_E_RIGHT (13, 2050, 10000000),
        MOD_E_3 (14, 10000, 10000000),
        MOD_F_END (15, 500, 10000000);
        int num, time, startBy;
        private Step (int num, int time, int startBy) {
            this.num = num;
            this.time = time;
            this.startBy = startBy;
        }
    }
    public Step currentStep;
    public Step nextStep;
    public StartingLocation startingLocation = StartingLocation.RED_NEAR;

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime steptime = new ElapsedTime();
    int nextSteps = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        northEastMotor = hardwareMap.get(DcMotorEx.class, "northeastMotor");
        northWestMotor = hardwareMap.get(DcMotorEx.class, "northwestMotor");
        southEastMotor = hardwareMap.get(DcMotorEx.class, "southeastMotor");
        southWestMotor = hardwareMap.get(DcMotorEx.class, "southwestMotor");
        northWestMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        northEastMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        DriveTrain myDrive = new DriveTrain(northWestMotor, northEastMotor, southWestMotor, southEastMotor, 3, 0);

        extend = hardwareMap.get(DcMotorEx.class, "length");
        shoulder = hardwareMap.get(DcMotorEx.class, "lift");
        encoder = hardwareMap.get(DcMotorEx.class, "measure");
        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extend.setDirection(DcMotorSimple.Direction.REVERSE);
        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        encoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        claw = hardwareMap.get(CRServo.class, "clawServo");
        wrist = hardwareMap.get(Servo.class, "wristServo");

        magnet = hardwareMap.get(TouchSensor.class, "magneto");

        scrollArm = new DrawerSlide(shoulder, extend, encoder, wrist, claw, magnet);

        //init webcam for color detection
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new UniversalColorDeterminationPipeline(new int[] {100, 220}, new int[] {100, 120}, 20, 20, CR, CB, TOLERANCE);
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

        initCamera();
        locator = new AprilTagLocator();

        //Init loop instead of wait for start
        //Module 1
        wrist.setPosition(-1);
        while (!isStarted() && !isStopRequested())
        {
            telemetry.addData("Realtime analysis", pipeline.getRegion());
            telemetry.addData("Realtime color", pipeline.getColor());
            elementPos = pipeline.getRegion();
            telemetry.update();
            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }

        snapshotColor = UniversalColorDeterminationPipeline.ElementColor.RED;//ePipeline.getColor();
        if (snapshotColor == UniversalColorDeterminationPipeline.ElementColor.RED) {
            startingLocation = (NEAR) ? StartingLocation.RED_NEAR : StartingLocation.RED_FAR;
            switch (elementPos) {
                case -1:
                    targetTag = 4;
                    break;
                case 0:
                    targetTag = 5;
                    break;
                case 1:
                    targetTag = 6;
                    break;
            }
        } else if (snapshotColor == UniversalColorDeterminationPipeline.ElementColor.BLUE) {
            startingLocation = (NEAR) ? StartingLocation.BLUE_NEAR : StartingLocation.BLUE_FAR;
            switch (elementPos) {
                case -1:
                    targetTag = 1;
                    break;
                case 0:
                    targetTag = 2;
                    break;
                case 1:
                    targetTag = 3;
                    break;
            }
        }
        telemetry.addData("Starting Location: ", startingLocation);
        telemetry.update();
        currentStep = Step.MOD_B_1;
        nextStep = Step.MOD_B_2;
        runtime.reset();
        steptime.reset();
        while (opModeIsActive()) {
            //Time stepper system
            if (steptime.milliseconds() >= currentStep.time) {
                stepNext();
                // || runtime.milliseconds() >= nextStep.startBy
            }
            telemetry.addData("bwoken:", (nextStep.num != currentStep.num + 1));
            switch (currentStep.num) {
                //MODULE B
                case 0:
                    myDrive.setDrivePower(0, 0, -0.05f, 0.75f);
                    shoulder.setPower(0.2);
                    break;
                case 1:
                    myDrive.setDrivePower(0, -0.375f, 0.05f, 0);
                    shoulder.setPower(0);
                    break;
                case 2:
                    myDrive.setDrivePower(0, 0, 0, 0);
                    break;
                case 3:
                    myDrive.setDrivePower(0, 0, 0, 0);
                    claw.setPower(-1);
                    break;
                /*case 4:
                    claw.setPower(1);
                    wrist.setPosition(-1);
                    shoulder.setPower(0.7);
                    break;
                case 5:
                    claw.setPower(0);
                    shoulder.setPower(0);
                    switch (elementPos) {
                        case -1:
                            myDrive.setDrivePower(0, 0, 0.4f, 0);
                            break;
                        case 0:
                            myDrive.setDrivePower(0, -0.15f, -0.2f, 0);
                            break;
                        case 1:
                            myDrive.setDrivePower(0, 0, 0, 0);
                            stepNext();
                            break;
                    }
                    break;
                case 6:
                    myDrive.setDrivePower(0, 0, 0, 0);
                    stepNext();
                    break;
                    //Module C
                case 7:
                    myDrive.setDrivePower(0, -0.75f, 0.1f, 0);
                    break;
                case 8:
                    if (snapshotColor == UniversalColorDeterminationPipeline.ElementColor.RED) {
                        myDrive.setDrivePower(0, 0, 0.5f, 0);
                    } else {
                        myDrive.setDrivePower(0, 0, -0.5f, 0);
                    }
                    break;
                case 9:
                case 10:
                    myDrive.setDrivePower(0, -0.75f, 0.1f, 0);
                    break;
                case 11:
                    myDrive.setDrivePower(0, 0, 0, 0);
                    stepNext();
                    break;
                case 12:
                    claw.setPower(-1);
                    /*if (snapshotColor == ElementDeterminationPipeline.ElementColor.RED) {
                        myDrive.setDrivePower(0, 0, -0.08f, 0.6f);
                    } else {
                        myDrive.setDrivePower(0, 0, 0.08f, -0.6f);
                    }
                    stepNext();*/
                   /* break;
                case 13:
                    telemetry.addData("tag: ", locator.findTag(targetTag));
                    if (locator.findTag(targetTag)) {
                        stepNext();
                    }
                    break;
                case 14:
                    myDrive.setDrivePower(0, 0, 0, 0);
                    stepNext();
                    break;
                case 15:
                    myDrive.setDrivePower(0, 0, 0, 0);
                    break;*/

            }
            telemetry.addData("Starting Location: ", startingLocation);
            telemetry.addData("Step: ", currentStep);
            telemetry.addData("Step num: ", currentStep.num);
            telemetry.addData("next steps: ", nextSteps);
            telemetry.update();
            //breaks after last step
            if (currentStep.num >= 4) {break;}
            //breaks if all time is gone
            if (runtime.milliseconds() >= 30000) {break;}
        }
    }

    public boolean stepNext () {
        nextSteps++;
        steptime.reset();
        currentStep = nextStep;
        if (currentStep != Step.MOD_F_END) {
            nextStep = Step.values()[currentStep.num + 1];
            nextStep = (nextStep == Step.MOD_D) ? ((startingLocation == StartingLocation.BLUE_NEAR || startingLocation == StartingLocation.RED_NEAR) ? Step.MOD_C_END : Step.MOD_D) : nextStep;
            if (nextStep.num == 12) {
                if (currentStep.num != 12) {
                    switch (elementPos) {
                        case -1:
                            nextStep = Step.MOD_E_LEFT;
                            break;
                        case 0:
                            nextStep = Step.MOD_E_CENTER;
                            break;
                        case 1:
                            nextStep = Step.MOD_E_RIGHT;
                            break;
                    }
                } else {
                    nextStep = Step.MOD_E_3;
                }
            }
            return true;
        } else {
            return false;
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
}
