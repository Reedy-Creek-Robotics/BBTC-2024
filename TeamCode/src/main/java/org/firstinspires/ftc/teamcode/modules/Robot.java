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
            PINCHER_CLOSED = 0.5,
            PINCHER_OPEN = 0.75,
            CLAW_CLOSED = 0.45,
            CLAW_OPEN = 0.0,
            BASKET_UP = 0.58,
            BASKET_DOWN = 0.24,
            INTAKE_SLIDE_OUT = 0.1,
            INTAKE_SLIDE_IN = 0.67;
    public static final int
            OUTTAKE_SLIDE_PREP_HANG = 1900,
            OUTTAKE_SLIDE_HANG = 800,
            OUTTAKE_SLIDE_UP = 3200,
            OUTTAKE_SLIDE_DOWN = 100;

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
               //Servo claw,
               Telemetry telemetry,
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
        //this.claw = claw;
        this.telemetry = telemetry;
        this.opMode = opMode;
    }

    public void runIntake(RunStates RunState, double speed){
            intakeArm.setPosition(RunState.getArmPos());
            pincherRotator.setPosition(RunState.getPincherRotatorPos());

            if(RunState.getIntakeRotatorPos() >= 0){
                intakeRotator.setPosition(RunState.getIntakeRotatorPos());
            }
            intakeRotator.setPosition(RunState.getIntakeRotatorPos());

            if(RunState.getSlidePos() >= 0){
                intakeSlide.setPosition(RunState.getSlidePos());
            }

            basket.setPosition(RunState.getBasketPos());
            pincher.setPosition(RunState.isPincherOpen() ? PINCHER_OPEN : PINCHER_CLOSED);
            //claw.setPosition(RunState.isClawOpen() ? CLAW_OPEN : CLAW_CLOSED);
            outtakeSlideRight.setTargetPosition(RunState.getOuttakeSlidePos());
            outtakeSlideLeft.setTargetPosition(RunState.getOuttakeSlidePos());
            outtakeSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            outtakeSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            outtakeSlideRight.setPower(speed);
            outtakeSlideLeft.setPower(speed);
    }
}
