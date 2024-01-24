package org.firstinspires.ftc.teamcode;

import com.epra.Controller;
import com.epra.DriveTrain;
import com.epra.pipelines.ElementDeterminationPipeline;
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
public class AutoRedNear extends LinearOpMode {
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
    private ElementDeterminationPipeline pipeline;
    ElementDeterminationPipeline.ElementPosition snapshotAnalysis = ElementDeterminationPipeline.ElementPosition.LEFT; // default
    ElementDeterminationPipeline.ElementColor snapshotColor = ElementDeterminationPipeline.ElementColor.BLUE;

    private ElapsedTime runtime = new ElapsedTime();

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

        //init webcam for color detection
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new ElementDeterminationPipeline(0, 130, 260, 180, 165, 180, 30, 60);
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

        //Init loop instead of wait for start
        while (!isStarted() && !isStopRequested())
        {
            if (pipeline.getPosition() != null) {
                telemetry.addData("Realtime analysis", pipeline.getPosition());
                telemetry.addData("Realtime color", pipeline.getColor());
                telemetry.update();
            }
            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
        snapshotAnalysis = pipeline.getPosition();
        snapshotColor = pipeline.getColor();
        while (opModeIsActive()) {
            //stepper

        }
    }
}
