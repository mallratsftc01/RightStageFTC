package org.firstinspires.ftc.teamcode;

import com.epra.Controller;
import com.epra.DriveTrain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class TempTimeAutoRedFar extends LinearOpMode {
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
                    myDrive.setDrivePower(0, 0.5f, 0, 0);
                    stepTime = 2.2;
                    break;
                case 7:
                    myDrive.setDrivePower(0, 0, 0.1f, -0.5f);
                    stepTime = 7;
                    break;
                case 8:
                    myDrive.setDrivePower(0, 0.5f, 0, 0);
                    stepTime = 1;
                    break;
            }
            if (runtime.seconds() >= stepTime) {
                step++;
                runtime.reset();
            }
            telemetry.addData("Step: ", step);
            telemetry.addData("Seconds Elapsed:", runtime.seconds());
            telemetry.update();
            if (step > 8
            ) {break;}
        }
    }
}