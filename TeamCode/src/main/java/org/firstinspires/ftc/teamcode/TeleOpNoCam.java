package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.modules.Robot;
import static org.firstinspires.ftc.teamcode.modules.Robot.*;
import static org.firstinspires.ftc.teamcode.modules.Robot.BASKET_DOWN;
import static org.firstinspires.ftc.teamcode.modules.Robot.BASKET_UP;
import static org.firstinspires.ftc.teamcode.modules.Robot.INTAKE_SLIDE_IN;
import static org.firstinspires.ftc.teamcode.modules.Robot.INTAKE_SLIDE_OUT;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.*;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.*;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.modules.RunStates;

@TeleOp(name = "Tele-Op NO CAM")
public class TeleOpNoCam extends LinearOpMode {
    ElapsedTime buttonDebounce;
    ElapsedTime timer;
    ElapsedTime transferTimer;

    // Delay between button presses in ms
    static int buttonDelay = 250;

    double ly1;
    double lx1;
    double rx1;
    double rotX;
    double rotY;
    double pincherRotatorPos;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();

    DcMotor driveFrontLeft;
    DcMotor driveFrontRight;
    DcMotor driveBackLeft;
    DcMotor driveBackRight;
    DcMotor outtakeSlideRight; //
    DcMotor outtakeSlideLeft; //

    Servo pincher; //
    Servo intakeSlide; //
    Servo intakeArm; //
    Servo pincherRotator; //
    Servo intakeRotator; //
    Servo claw; //
    Servo basket; //

    Robot bot;

    IMU imu;

    boolean basketUp = false;
    boolean clawOpen = false;
    boolean intakeSlideOut = false;
    boolean outtakeSlideUp = false;
    boolean robotDrive = true;
    boolean transferring = false;

    @Override
    public void runOpMode()  {
        initHardware();
        buttonDebounce = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        transferTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


        telemetry.addLine("> PRESS START");
        waitForStart();

        bot.runIntake(RunStates.DEFAULT, 1);

        while(opModeIsActive()) {

            processVariableUpdates();
            if(robotDrive) {processDrivingRobot();}
            else {processDrivingField();}
            processControl();
            processTelemetry();

        }
    }
    private void processDrivingRobot(){
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
    private void processDrivingField(){
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        rotX = lx1 * Math.cos(-botHeading) - ly1 * Math.sin(-botHeading);
        rotY = lx1 * Math.sin(-botHeading) + ly1 * Math.cos(-botHeading);

        rotX *= 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx1), 1);
        double frontLeftPower = (rotY + rotX + rx1) / denominator;
        double backLeftPower = (rotY - rotX + rx1) / denominator;
        double frontRightPower = (rotY - rotX - rx1) / denominator;
        double backRightPower = (rotY + rotX - rx1) / denominator;

        driveFrontLeft.setPower(frontLeftPower);
        driveBackLeft.setPower(backLeftPower);
        driveFrontRight.setPower(frontRightPower);
        driveBackRight.setPower(backRightPower);
    }

    private void processControl() {
        if (gamepad1.options) {
            imu.resetYaw();
        }

        if(gamepad1.left_bumper && buttonDebounce.milliseconds() > buttonDelay){
            clawOpen = !clawOpen;
            buttonDebounce.reset();
        }

        if(gamepad1.right_bumper && buttonDebounce.milliseconds() > buttonDelay){
            basketUp = !basketUp;
            buttonDebounce.reset();
        }

        if(gamepad1.a && buttonDebounce.milliseconds() > buttonDelay){
            bot.runIntake(RunStates.GRAB, 1);
            buttonDebounce.reset();
        }

        if(gamepad1.dpad_up && buttonDebounce.milliseconds() > buttonDelay){
            intakeSlideOut = true;
            buttonDebounce.reset();
        }

        if(gamepad1.dpad_down && buttonDebounce.milliseconds() > buttonDelay){
            intakeSlideOut = false;
            buttonDebounce.reset();
        }

        if(gamepad1.left_stick_button && buttonDebounce.milliseconds() > buttonDelay){
            outtakeSlideUp = !outtakeSlideUp;
            if(outtakeSlideUp == false){
                basketUp = false;
            }
            transferTimer.reset();
            buttonDebounce.reset();
        }



        if(gamepad1.dpad_left && buttonDebounce.milliseconds() > 100){
            pincherRotatorPos = 0.35;
            buttonDebounce.reset();
        }

        if(gamepad1.dpad_right && buttonDebounce.milliseconds() > 100){
            pincherRotatorPos = 0.65;
            buttonDebounce.reset();
        }

        if(gamepad1.x && buttonDebounce.milliseconds() > buttonDelay){
            bot.runIntake(RunStates.DEFAULT, 1);
            buttonDebounce.reset();
        }

        if(gamepad1.b && buttonDebounce.milliseconds() > buttonDelay){
            bot.runIntake(RunStates.HOLD, 1);
            intakeSlideOut = false;
            pincherRotatorPos = 0.65;
            basketUp = false;
            outtakeSlideUp = false;
            buttonDebounce.reset();
            transferring = true;
        }

        if(transferring && transferTimer.milliseconds() > 500){
            clawOpen = true;
        }

        if(transferring && transferTimer.milliseconds() > 750) {
            bot.runIntake(RunStates.DEFAULT, 1);
            transferring = false;
        }

        pincherRotator.setPosition(pincherRotatorPos);
        intakeSlide.setPosition(intakeSlideOut ? INTAKE_SLIDE_OUT : INTAKE_SLIDE_IN);
        pincher.setPosition(clawOpen ? PINCHER_OPEN : PINCHER_CLOSED);
        basket.setPosition(basketUp ? BASKET_UP : BASKET_DOWN);
        outtakeSlideRight.setTargetPosition(outtakeSlideUp ? 3200 : 0);
        outtakeSlideLeft.setTargetPosition(outtakeSlideUp ? 3200 : 0);
        outtakeSlideRight.setMode(RUN_TO_POSITION);
        outtakeSlideLeft.setMode(RUN_TO_POSITION);
        outtakeSlideRight.setPower(1);
        outtakeSlideLeft.setPower(1);

    }

    private void processVariableUpdates() {
        ly1 = -gamepad1.left_stick_y;
        // We apply a 1.1 multiplier to the x-axis to make apply for strafing inaccuracies
        lx1 = gamepad1.left_stick_x * 1.1;
        rx1 = gamepad1.right_stick_x;

        /*if(gamepad1.start && buttonDebounce.milliseconds() > buttonDelay){
               robotDrive = !robotDrive;
        }*/

        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);
    }
    private void processTelemetry(){
        telemetry.addData("Basket Up", basketUp);
        telemetry.addData("Claw Open", clawOpen);
        telemetry.addData("Intake Out", intakeSlideOut);
        telemetry.addData("Outtake Up", outtakeSlideUp);
        telemetry.addData("Pincher Open", clawOpen);
        telemetry.addData("Driving Mode", robotDrive ? "Robot" : "Field");
        telemetry.addData("Outtake Slide Pos", outtakeSlideLeft.getCurrentPosition());
        telemetry.addData("Pincher Rotator Position", pincherRotatorPos);
        telemetry.update();
    }
    private void initHardware() {
        driveFrontLeft = hardwareMap.get(DcMotor.class, "driveFrontLeft");
        driveFrontLeft.setMode(RUN_WITHOUT_ENCODER);
        driveFrontLeft.setZeroPowerBehavior(BRAKE);
        driveFrontLeft.setDirection(REVERSE);

        driveFrontRight = hardwareMap.get(DcMotor.class, "driveFrontRight");
        driveFrontRight.setMode(RUN_WITHOUT_ENCODER);
        driveFrontRight.setZeroPowerBehavior(BRAKE);

        driveBackLeft = hardwareMap.get(DcMotor.class, "driveBackLeft");
        driveBackLeft.setMode(RUN_WITHOUT_ENCODER);
        driveBackLeft.setZeroPowerBehavior(BRAKE);
        driveBackLeft.setDirection(REVERSE);

        driveBackRight = hardwareMap.get(DcMotor.class, "driveBackRight");
        driveBackRight.setMode(RUN_WITHOUT_ENCODER);
        driveBackRight.setZeroPowerBehavior(BRAKE);

        outtakeSlideRight = hardwareMap.get(DcMotor.class, "outtakeSlideRight");
        outtakeSlideRight.setMode(STOP_AND_RESET_ENCODER);
        outtakeSlideRight.setMode(RUN_USING_ENCODER);
        outtakeSlideRight.setZeroPowerBehavior(BRAKE);

        outtakeSlideLeft = hardwareMap.get(DcMotor.class, "outtakeSlideLeft");
        outtakeSlideLeft.setMode(STOP_AND_RESET_ENCODER);
        outtakeSlideLeft.setMode(RUN_USING_ENCODER);
        outtakeSlideLeft.setZeroPowerBehavior(BRAKE);
        outtakeSlideLeft.setDirection(REVERSE);


        pincher = hardwareMap.get(Servo.class, "pincher");

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
                this
        );

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));
    }
}