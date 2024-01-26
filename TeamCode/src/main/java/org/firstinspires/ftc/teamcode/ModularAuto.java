package org.firstinspires.ftc.teamcode;

import com.epra.Controller;
import com.epra.DriveTrain;
import com.epra.pipelines.ElementDeterminationPipeline;
import com.epra.pipelines.UniversalColorDeterminationPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class ModularAuto extends LinearOpMode {
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

    private OpenCvWebcam webcam;
    private ElementDeterminationPipeline ePipeline;
    private UniversalColorDeterminationPipeline cPipeline;
    ElementDeterminationPipeline.ElementPosition snapshotAnalysis = ElementDeterminationPipeline.ElementPosition.LEFT; // default
    ElementDeterminationPipeline.ElementColor snapshotColor = ElementDeterminationPipeline.ElementColor.BLUE;
    int snapshotRegion = -1;
    UniversalColorDeterminationPipeline.ElementColor snapshotColorU = UniversalColorDeterminationPipeline.ElementColor.WHITE;
    private enum StartingLocation {
        RED_NEAR,
        RED_FAR,
        BLUE_NEAR,
        BLUE_FAR
    }
    public enum Step {
        MOD_B_1 (0, 1100, 0),
        MOD_B_2 (1, 550, 10000000),
        MOD_B_3 (2, 0, 10000000),
        MOD_B_4 (3, 0, 10000000),
        MOD_B_5 (4, 550, 10000000),
        MOD_B_END(5, 0, 1000000000),
        MOD_C_1 (6, 0, 10000000),
        MOD_D_1 (7, 0, 10000000),
        MOD_E_1 (8, 0, 10000000),
        MOD_F_END (9, 0, 10000000);
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

    @Override
    public void runOpMode() throws InterruptedException {
        northEastMotor = hardwareMap.get(DcMotorEx.class, "northeastMotor");
        northWestMotor = hardwareMap.get(DcMotorEx.class, "northwestMotor");
        southEastMotor = hardwareMap.get(DcMotorEx.class, "southeastMotor");
        southWestMotor = hardwareMap.get(DcMotorEx.class, "southwestMotor");
        northWestMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        northEastMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        DriveTrain myDrive = new DriveTrain(northWestMotor, northEastMotor, southWestMotor, southEastMotor, 3, 0);

        rightLift = hardwareMap.get(DcMotorEx.class, "length");
        leftLift = hardwareMap.get(DcMotorEx.class, "lift");
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);

        claw = hardwareMap.get(Servo.class, "clawServo");
        wrist = hardwareMap.get(Servo.class, "wristServo");

        controller1 = new Controller(gamepad1, 0.05F);
        controller2 = new Controller(gamepad2, 0.05F);

        magnet = hardwareMap.get(TouchSensor.class, "magneto");

        //init webcam for color detection
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        ePipeline = new ElementDeterminationPipeline(0, 130, 260, 180, 165, 180, 30, 60);
        cPipeline = new UniversalColorDeterminationPipeline(new int[] {0, 290}, new int [] {30, 30}, 30, 30, 128, 128, 32);
        webcam.setPipeline(cPipeline);
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

        //Init loop instead of wait for start
        //Module 1
        while (!isStarted() && !isStopRequested())
        {
            webcam.setPipeline(ePipeline);
            if (ePipeline.getPosition() != null) {
                telemetry.addData("Realtime analysis", ePipeline.getPosition());
                telemetry.addData("Realtime color", ePipeline.getColor());
            }
            /*webcam.setPipeline(cPipeline);
            if (cPipeline.getColor() != null) {
                telemetry.addData("Realtime region", cPipeline.getRegion());
                telemetry.addData("Realtime color", cPipeline.getColor());
            }*/
            telemetry.update();
            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }

        snapshotAnalysis = ePipeline.getPosition();
        snapshotColor = ePipeline.getColor();
        snapshotRegion = 0;//cPipeline.getRegion();
        snapshotColorU = UniversalColorDeterminationPipeline.ElementColor.WHITE;//cPipeline.getColor();
        if (snapshotColor == ElementDeterminationPipeline.ElementColor.RED) {
            startingLocation = (snapshotRegion == 0) ? StartingLocation.RED_NEAR : StartingLocation.RED_FAR;
        } else if (snapshotColor == ElementDeterminationPipeline.ElementColor.BLUE) {
            startingLocation = (snapshotRegion == 1) ? StartingLocation.BLUE_NEAR : StartingLocation.BLUE_FAR;
        }
        telemetry.addData("Starting Location: ", startingLocation);
        telemetry.update();
        currentStep = Step.MOD_B_1;
        nextStep = Step.MOD_B_2;
        runtime.reset();
        steptime.reset();
        while (opModeIsActive()) {
            //Time stepper system
            if (steptime.milliseconds() >= currentStep.time || runtime.milliseconds() >= nextStep.startBy) {
                stepNext();
            }
            switch (currentStep.num) {
                //MODULE B
                case 0:
                    myDrive.setDrivePower(0, -0.75f, 0.25f, 0);
                    break;
                case 1:
                    switch (snapshotAnalysis) {
                        case LEFT:
                            myDrive.setDrivePower(0, 0, -0.5f, 0);
                            break;
                        case RIGHT:
                            myDrive.setDrivePower(0, 0, 0.5f, 0);
                            break;
                        case CENTER:
                            myDrive.setDrivePower(0, 0, 0, 0);
                            stepNext();
                            break;
                    }
                    break;
                case 2:
                    myDrive.setDrivePower(0, 0, 0, 0);
                    //open claw
                    break;
                case 3:
                    //close claw
                    break;
                case 4:
                    switch (snapshotAnalysis) {
                        case LEFT:
                            myDrive.setDrivePower(0, 0, 0.5f, 0);
                            break;
                        case RIGHT:
                            myDrive.setDrivePower(0, 0, -0.5f, 0);
                            break;
                        case CENTER:
                            myDrive.setDrivePower(0, 0, 0, 0);
                            stepNext();
                            break;
                    }
                    break;
                case 5:
                    myDrive.setDrivePower(0, 0, 0, 0);
                    break;
            }
            telemetry.addData("Starting Location: ", startingLocation);
            telemetry.addData("Step: ", currentStep);
            telemetry.update();
            //breaks after last step
            if (currentStep.num >= 5) {break;}
            //breaks if all time is gone
            if (runtime.milliseconds() >= 30000) {break;}
        }
    }

    public boolean stepNext () {
        steptime.reset();
        currentStep = nextStep;
        if (currentStep != Step.MOD_F_END) {
            nextStep = Step.values()[currentStep.num + 1];
            nextStep = (nextStep == Step.MOD_C_1 && (startingLocation == StartingLocation.BLUE_FAR || startingLocation == StartingLocation.RED_FAR)) ? Step.MOD_D_1 : Step.MOD_C_1;
            nextStep = (nextStep == Step.MOD_D_1 && (startingLocation == StartingLocation.BLUE_NEAR || startingLocation == StartingLocation.RED_NEAR)) ? Step.MOD_E_1 : Step.MOD_D_1;
            return true;
        } else {
            return false;
        }
    }
}
