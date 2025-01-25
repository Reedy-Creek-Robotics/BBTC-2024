package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.modules.Robot;

import static org.firstinspires.ftc.teamcode.modules.Robot.PINCHER_OPEN;
import static org.firstinspires.ftc.teamcode.modules.Robot.PINCHER_CLOSED;
import static org.firstinspires.ftc.teamcode.modules.Robot.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.modules.Robot.CLAW_CLOSED;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.*;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.modules.RunStates;
import org.firstinspires.ftc.teamcode.modules.VisionPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;

@TeleOp(name = "Tele-Op Driving")
public class TeleOpDrive extends LinearOpMode {

    ElapsedTime buttonDebounce;
    ElapsedTime timer;
    VisionPipeline yellowVisionPipeline;
    VisionPipeline alliaceVisionPipeline;
    VisionPortal visionPortal;

    // Delay between button presses in ms
    static int buttonDelay = 250;

    // 0 = red, 1 = blue
    int alliance = 0;
    // 0 = Do not have specimen, 1 = Have specimen, go above high chamber, 2 = deposit high chamber
    int specimenState = 0;

    double ly1;
    double lx1;
    double rx1;
    double lt2;
    double rt2;
    double lt1;
    double rt1;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();

    DcMotor driveFrontLeft;
    DcMotor driveFrontRight;
    DcMotor driveBackLeft;
    DcMotor driveBackRight;
    DcMotor outtakeSlideRight;
    DcMotor outtakeSlideLeft;

    Servo pincher;
    Servo intakeSlide;
    Servo intakeArm;
    Servo pincherRotator;
    Servo intakeRotator;
    Servo basket;
    Servo claw;

    OpenCvCamera webcam1;

    Robot bot;

    boolean pincherOpen;
    boolean hangPrimed = false;
    boolean hangInitiated = false;
    boolean outtakingBasket = false;
    boolean outtakingChamber = false;
    boolean intakingWall = false;
    boolean droppingSample = false;
    boolean grabbing = false;

    //defining length of arms in servo linkage in inches
    double arm1 = 2.814920799212598;
    double arm2 = 10.078740275590551;
    double distanceArm1AboveArm2 = 0.34252;


    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        initOpenCv();
        buttonDebounce = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        telemetry.addLine("> PRESS START");
        waitForStart();

        while(opModeIsActive()) {

            processVariableUpdates();

            processDriving();
            processControl();
            processTelemetry();

        }
    }

    //ToDo Add a field centric driving method
    private void processDriving(){
        double denominator = Math.max(Math.abs(ly1) + Math.abs(lx1) + Math.abs(rx1), 1);
        double frontLeftPower = (ly1 + lx1 + rx1) / denominator;
        double backLeftPower = (ly1 - lx1 + rx1) / denominator;
        double frontRightPower = (ly1 - lx1 - rx1) / denominator;
        double backRightPower = (ly1 + lx1 - rx1) / denominator;

        driveFrontLeft.setPower(frontLeftPower);
        driveBackLeft.setPower(backLeftPower);
        driveFrontRight.setPower(frontRightPower);
        driveBackRight.setPower(backRightPower);
    }

    private void processControl() {

        if(gamepad1.right_stick_button && buttonDebounce.milliseconds() > buttonDelay){
            bot.runIntake(RunStates.DEFAULT, 1);
        }

        // We check if the debounce is greater than the button delay to avoid one press being registered as many
        if(gamepad1.dpad_right && buttonDebounce.milliseconds() > buttonDelay) {
            autoIntakeSample(alliance == 0 ? "red" : "blue");
        }

        if(gamepad1.b && buttonDebounce.milliseconds() > buttonDelay) {
            autoIntakeSample("yellow");
        }

        if(gamepad1.dpad_down && buttonDebounce.milliseconds() > buttonDelay){
            grabbing = false;
            timer.reset();
            bot.runIntake(RunStates.PREPARE_WALL, 1);
            intakingWall = true;
        }

        if(gamepad1.dpad_left && buttonDebounce.milliseconds() > buttonDelay){
            bot.runIntake(RunStates.DEFAULT, 1);
            timer.reset();
            droppingSample = true;
        }

        if(gamepad1.a && buttonDebounce.milliseconds() > buttonDelay) {
            bot.runIntake(RunStates.TRANSFER, 1);
            bot.runIntake(RunStates.DEFAULT, 1);
            bot.runIntake(RunStates.BASKET_PREPARE, 1);
            outtakingBasket = true;
        }

        if(gamepad1.dpad_up && buttonDebounce.milliseconds() > buttonDelay){
            specimenState++;
            if(specimenState > 2){
                specimenState = 0;
            }
        }

        if(outtakingBasket){ outtakeBasket(); }
        if(specimenState > 0){ outtakeChamber(); }
        if(intakingWall) { intakeWall(); }
        if(droppingSample) { dropSample(); }

        if (pincherOpen) {
            pincher.setPosition(PINCHER_OPEN);
        } else {
            pincher.setPosition(PINCHER_CLOSED);
        }
    }

    //TODO get position from roadrunner so we can give to VisionPipelines
    private void processVariableUpdates() {
        ly1 = -gamepad1.left_stick_y;
        // We apply a 1.1 multiplier to the x-axis to make apply for strafing inaccuracies
        lx1 = gamepad1.left_stick_x * 1.1;
        rx1 = gamepad1.right_stick_x;

        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        if (gamepad1.start && buttonDebounce.milliseconds() > buttonDelay) {
            if (alliance == 0) {
                alliance = 1;
            } else {
                alliance = 0;
            }
        }
    }

    private void processTelemetry(){
        telemetry.addData("Alliance", alliance == 0 ? "Red" : "Blue");
        telemetry.addData("Specimen State", specimenState);
        telemetry.update();
    }

    private void initHardware() {
        driveFrontLeft = hardwareMap.get(DcMotor.class, "driveFrontLeft");
        driveFrontLeft.setMode(STOP_AND_RESET_ENCODER);
        driveFrontLeft.setMode(RUN_USING_ENCODER);
        driveFrontLeft.setZeroPowerBehavior(BRAKE);
        driveFrontLeft.setDirection(REVERSE);

        driveFrontRight = hardwareMap.get(DcMotor.class, "driveFrontRight");
        driveFrontRight.setMode(STOP_AND_RESET_ENCODER);
        driveFrontRight.setMode(RUN_USING_ENCODER);
        driveFrontRight.setZeroPowerBehavior(BRAKE);

        driveBackLeft = hardwareMap.get(DcMotor.class, "driveBackLeft");
        driveBackLeft.setMode(STOP_AND_RESET_ENCODER);
        driveBackLeft.setMode(RUN_USING_ENCODER);
        driveBackLeft.setZeroPowerBehavior(BRAKE);
        driveBackLeft.setDirection(REVERSE);

        driveBackRight = hardwareMap.get(DcMotor.class, "driveBackRight");
        driveBackRight.setMode(STOP_AND_RESET_ENCODER);
        driveBackRight.setMode(RUN_USING_ENCODER);
        driveBackRight.setZeroPowerBehavior(BRAKE);

        outtakeSlideRight = hardwareMap.get(DcMotor.class, "outtakeSlideRight");
        outtakeSlideRight.setMode(STOP_AND_RESET_ENCODER);
        outtakeSlideRight.setMode(RUN_USING_ENCODER);
        outtakeSlideRight.setZeroPowerBehavior(BRAKE);
        outtakeSlideRight.setDirection(REVERSE);

        outtakeSlideLeft = hardwareMap.get(DcMotor.class, "outtakeSlideLeft");
        outtakeSlideLeft.setMode(STOP_AND_RESET_ENCODER);
        outtakeSlideLeft.setMode(RUN_USING_ENCODER);
        outtakeSlideLeft.setZeroPowerBehavior(BRAKE);


        pincher = hardwareMap.get(Servo.class, "pincher");
        pincherOpen = false;

        intakeArm = hardwareMap.get(Servo.class, "intakeArm");

        pincherRotator = hardwareMap.get(Servo.class, "pincherRotator");

        intakeRotator = hardwareMap.get(Servo.class, "intakeRotator");

        intakeSlide = hardwareMap.get(Servo.class, "intakeSlide");

        basket = hardwareMap.get(Servo.class, "basket");

        //claw = hardwareMap.get(Servo.class, "claw");

        this.bot = new Robot(
                driveFrontLeft,
                driveBackLeft,
                driveBackRight,
                driveFrontRight,
                outtakeSlideRight,
                outtakeSlideLeft,
                intakeArm,
                pincherRotator,
                intakeRotator,
                intakeSlide,
                basket,
                pincher,
                //claw,
                telemetry,
                webcam1,
                this
        );
    }

    private void initOpenCv(){
        yellowVisionPipeline = new VisionPipeline(2);
        alliaceVisionPipeline = new VisionPipeline(alliance);
        visionPortal = new VisionPortal.Builder().
                addProcessor(yellowVisionPipeline)
                .addProcessor(alliaceVisionPipeline)
                .setCameraResolution(new android.util.Size(1920, 1080))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam1"))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setAutoStartStreamOnBuild(true)
                .enableLiveView(true)
                .build();
    }


    // Action Methods

    boolean slidesPastTolerance(){
        if(
                outtakeSlideLeft.getTargetPosition() - outtakeSlideLeft.getCurrentPosition() < 250 ||
                outtakeSlideLeft.getTargetPosition() - outtakeSlideLeft.getCurrentPosition() > 250 ||
                outtakeSlideRight.getTargetPosition() - outtakeSlideRight.getCurrentPosition() < 250 ||
                outtakeSlideRight.getTargetPosition() - outtakeSlideRight.getCurrentPosition() > 250

        ) { return true; } else { return false; }

    }

    void dropSample (){
            if(timer.milliseconds() > 1000) {
                bot.runIntake(RunStates.DROP_FRONT, 1);
                timer.reset();
                droppingSample = false;
            }
    }

    void autoIntakeSample(String color){
         //ToDo Find the math required to get the offset from the robot to the sample, taking into account the extra distance at the end of the intake slide
        Point sCentroid;
        double sAngle;
         if(color == "yellow"){
             //ToDo Get the location of the nearest yellow sample
             if(!yellowVisionPipeline.nothingThere){
                 sCentroid = yellowVisionPipeline.centroid;
                 sAngle = yellowVisionPipeline.angleOfRotation;
             }
         } else if(color == "blue") {
             //ToDo Get the location of the nearest blue sample
             sCentroid = alliaceVisionPipeline.centroid;
             sAngle = alliaceVisionPipeline.angleOfRotation;
         } else {
             //ToDo Get the location of the nearest red sample
             sCentroid = alliaceVisionPipeline.centroid;
             sAngle = alliaceVisionPipeline.angleOfRotation;
         }
         double targetX = 0;
         double targetY = 0;

         bot.runIntake(RunStates.HOLD, 1);
    }

    void outtakeBasket(){
        boolean dropping = false;
        if(!slidesPastTolerance()){
            bot.runIntake(RunStates.BASKET_DROP, 1);
            dropping = true;
            timer.reset();
        }
        if(dropping && timer.milliseconds() > 1500){
            bot.runIntake(RunStates.DEFAULT, 1);
            outtakingBasket = false;
        }
    }

    void outtakeChamber(){
        if(specimenState == 1){
            bot.runIntake(RunStates.PREPARE_CHAMBER, 1);
        } else if(specimenState == 2){
            bot.runIntake(RunStates.SCORE_CHAMBER, 1);
        }
    }

    void intakeWall() {
        if (timer.milliseconds() > 1000) {
            bot.runIntake(RunStates.GRAB_WALL, 1);
            grabbing = true;
            timer.reset();
        }
        if (grabbing && timer.milliseconds() > 500) {
            bot.runIntake(RunStates.DEFAULT, 1);
            grabbing = false;
            intakingWall = false;
        }
    }

    void transferSample(){
        bot.runIntake(RunStates.TRANSFER, 1);
    }

    private double getLinkageAngle(double m){
        return 180-(Math.acos((((Math.pow(m,2)-Math.pow(arm2,2)+ Math.pow(distanceArm1AboveArm2,2))/
                                                arm1)  +arm1)/
                                   2*Math.sqrt(Math.pow(distanceArm1AboveArm2, 2)+ Math.pow(m,2))));
    }
}