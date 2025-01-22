package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.easyopencv.OpenCvCamera;

public class Robot {
    private DcMotor driveFrontLeft;
    private DcMotor driveFrontRight;
    private DcMotor driveBackRight;
    private DcMotor driveBackLeft;
    private DcMotor outtakeSlideRight;
    private DcMotor outtakeSlideLeft;

    private Servo intakeArm;
    private Servo pincherRotator;
    private Servo intakeRotator;
    private Servo intakeSlide;
    private Servo basket;
    private Servo pincher;
    private Servo claw;

    private OpenCvCamera webcam1;

    Telemetry telemetry;
    LinearOpMode opMode;

    public static final double
            PINCHER_CLOSED = 0,
            PINCHER_OPEN = 0,
            CLAW_CLOSED = 0,
            CLAW_OPEN = 0;

    public Robot(
            DcMotor driveFrontLeft,
            DcMotor driveBackLeft,
            DcMotor driveBackRight,
            DcMotor driveFrontRight,
            DcMotor outtakeSlideRight,
            DcMotor outtakeSlideLeft,
               Servo intakeArm,
               Servo pincherRotator,
               Servo intakeRotator,
               Servo intakeSlide,
               Servo basket,
               Servo pincher,
               Servo claw,
               Telemetry telemetry,
               OpenCvCamera webcam1,
               LinearOpMode opMode

    ){
        this.driveFrontLeft = driveFrontLeft;
        this.driveBackLeft = driveBackLeft;
        this.driveBackRight = driveBackRight;
        this.driveFrontRight = driveFrontRight;
        this.outtakeSlideRight = outtakeSlideRight;
        this.outtakeSlideLeft = outtakeSlideLeft;
        this.intakeArm = intakeArm;
        this.pincherRotator = pincherRotator;
        this.intakeRotator = intakeRotator;
        this.intakeSlide = intakeSlide;
        this.basket = basket;
        this.pincher = pincher;
        this.claw = claw;
        this.telemetry = telemetry;
        this.webcam1 = webcam1;
        this.opMode = opMode;
    }

    public void runIntake(RunStates RunStates, double speed){
            intakeArm.setPosition(RunStates.getArmPos());
            pincherRotator.setPosition(RunStates.getPincherRotatorPos());
            intakeRotator.setPosition(RunStates.getIntakeRotatorPos());
            if(RunStates.getSlidePos() >= 0){
                intakeSlide.setPosition(RunStates.getSlidePos());
            }
            basket.setPosition(RunStates.getBasketPos());
            pincher.setPosition(RunStates.isPincherOpen() ? PINCHER_OPEN : PINCHER_CLOSED);
            claw.setPosition(RunStates.isClawOpen() ? CLAW_OPEN : CLAW_CLOSED);
            outtakeSlideRight.setTargetPosition(RunStates.getOuttakeSlidePos());
            outtakeSlideLeft.setTargetPosition(-RunStates.getOuttakeSlidePos());
            outtakeSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            outtakeSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            outtakeSlideRight.setPower(speed);
            outtakeSlideLeft.setPower(speed);
    }
}
