package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.modules.Robot;

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
import org.openftc.easyopencv.OpenCvCamera;

@TeleOp(name = "Tele-Op Driving")
public class TeleOpDrive extends LinearOpMode {

    ElapsedTime buttonDebounce;
    ElapsedTime timer;

    static double PINCHER_OPEN = 0;
    static double PINCHER_CLOSED = 0;
    static double CLAW_OPEN = 0;
    static double CLAW_CLOSED = 0;

    // Delay between button presses in ms
    static int buttonDelay = 250;

    // 0 = red, 1 = blue
    int alliance = 0;

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
    boolean grabbing = false;

    /*
    ToDo Get intakePositions working for setting the positions of all devices

    ToDo Set up automatic sample intake and transfer
    ToDo Set up automatic sample outtake

    ToDo Set up automatic specimen outtake
     */

    @Override
    public void runOpMode() throws InterruptedException {

        initHardware();

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
        if(gamepad1.b && buttonDebounce.milliseconds() > buttonDelay) {
            autoIntakeSample(alliance == 0 ? "red" : "blue");
        }

        if(gamepad1.dpad_down && buttonDebounce.milliseconds() > buttonDelay){
            grabbing = false;
            timer.reset();
            bot.runIntake(RunStates.PREPARE_WALL, 1);
            intakeWall();
        }

        if(gamepad1.a && buttonDebounce.milliseconds() > buttonDelay) {
            bot.runIntake(RunStates.TRANSFER, 1);
            bot.runIntake(RunStates.DEFAULT, 1);
            bot.runIntake(RunStates.BASKET_PREPARE, 1);
            outtakingBasket = true;
        }

        if(outtakingBasket){ outtakeBasket(); }
        if(outtakingChamber){ outtakeChamber(); }

        if (pincherOpen) {
            pincher.setPosition(PINCHER_OPEN);
        } else {
            pincher.setPosition(PINCHER_CLOSED);
        }
    }

    private void processVariableUpdates() {
        ly1 = -gamepad1.left_stick_y;
        // We apply a 1.1 multiplier to the x-axis to make apply for strafing inaccuracies
        lx1 = gamepad1.left_stick_x * 1.1;
        rx1 = gamepad1.right_stick_x;

        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        if(gamepad1.back && buttonDebounce.milliseconds() > buttonDelay){
            // Set alliance to red
            alliance = 0;
        } else if(gamepad1.start && buttonDebounce.milliseconds() > buttonDelay){
            // Ser alliance to blue
            alliance = 1;
        }
    }

    private void processTelemetry(){
        telemetry.addData("Alliance", alliance == 0 ? "Red" : "Blue");
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

        claw = hardwareMap.get(Servo.class, "claw");

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
                claw,
                telemetry,
                webcam1,
                this
        );
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

    //void dropSample

    void autoIntakeSample(String color){
         //ToDo Find the math required to get the offset from the robot to the sample, taking into account the extra distance at the end of the intake slide

         if(color == "yellow"){
             //ToDo Get the location of the nearest yellow sample
         } else if(color == "blue") {
             //ToDo Get the location of the nearest blue sample
         } else {
             //ToDo Get the location of the nearest red sample
         }
         double targetX = 0;
         double targetY = 0;

         bot.runIntake(RunStates.HOLD, 1);
    }

    void intakeSpecimen(){
        bot.runIntake(RunStates.PREPARE_WALL, 1);
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

    void intakeWall() {
        if (timer.milliseconds() > 1000) {
            bot.runIntake(RunStates.GRAB_WALL, 1);
            grabbing = true;
            timer.reset();
        }
        if (grabbing && timer.milliseconds() > 500) {
            bot.runIntake(RunStates.DEFAULT, 1);
            grabbing = false;
        }
    }

     void outtakeChamber(){

    }

    void transferSample(){
        bot.runIntake(RunStates.TRANSFER, 1);
    }
}