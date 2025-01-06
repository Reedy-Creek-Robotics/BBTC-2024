package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class  TeleOpDrive extends LinearOpMode {

    ElapsedTime speedFactorDebounce;
    ElapsedTime intakeSpeedFactorDebounce;
    ElapsedTime wristPositionDebounce;

    double speedFactor = 0.7;
    double intakeSpeedFactor = 0.5;
    double ly1;
    double lx1;
    double rx1;

    Servo intakeArm;
    Servo intakeRotator;
    Servo pincherRotator;
    Servo intakeSlide;
    Servo pincher;
    Servo claw;
    Servo basket;

    DcMotor driveFrontLeft;
    DcMotor driveFrontRight;
    DcMotor driveBackLeft;
    DcMotor driveBackRight;
    DcMotor outtakeSlide1;
    DcMotor outtakeSlide2;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    static final int buttonDelay = 250;

    @Override
    public void runOpMode() throws InterruptedException {

        speedFactorDebounce = new ElapsedTime();
        intakeSpeedFactorDebounce = new ElapsedTime();
        wristPositionDebounce = new ElapsedTime();

        initHardware();

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

        driveFrontLeft.setPower(frontLeftPower * speedFactor);
        driveBackLeft.setPower(backLeftPower * speedFactor);
        driveFrontRight.setPower(frontRightPower * speedFactor);
        driveBackRight.setPower(backRightPower * speedFactor);
    }
    private void processVariableUpdates() {
        ly1 = -gamepad1.left_stick_y;
        lx1 = gamepad1.left_stick_x * 1.1;
        rx1 = gamepad1.right_stick_x;

        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        if (gamepad1.dpad_up && (speedFactorDebounce.milliseconds() >= buttonDelay)) {
            speedFactorDebounce.reset();
            speedFactor += 0.1;
        }

        if (gamepad1.dpad_down && (speedFactorDebounce.milliseconds() >= buttonDelay)) {
            speedFactorDebounce.reset();
            speedFactor -= 0.1;
        }

        if (speedFactor > 1) {
            speedFactor = 1;
        } else if (speedFactor <= 0) {
            speedFactor = 0.1;
        }

        if (gamepad2.dpad_right && (intakeSpeedFactorDebounce.milliseconds() >= buttonDelay)) {
            intakeSpeedFactorDebounce.reset();
            intakeSpeedFactor += 0.1;
        }

        if (gamepad2.dpad_left && (intakeSpeedFactorDebounce.milliseconds() >= buttonDelay)) {
            intakeSpeedFactorDebounce.reset();
            intakeSpeedFactor -= 0.1;
        }

        if (intakeSpeedFactor > 1) {
            intakeSpeedFactor = 1;
        } else if (intakeSpeedFactor <= 0) {
            intakeSpeedFactor = 0.1;
        }
    }

    private void processControl(){

    }
    private void processTelemetry(){

        telemetry.update();

    }

    private void initHardware() {
        driveFrontLeft = hardwareMap.get(DcMotor.class, "driveFrontLeft");
        driveFrontLeft.setMode(RUN_WITHOUT_ENCODER);
        driveFrontLeft.setZeroPowerBehavior(BRAKE);
        driveFrontLeft.setDirection(REVERSE);

        driveFrontRight = hardwareMap.get(DcMotor.class, "driveFrontRight");
        driveFrontRight.setMode(STOP_AND_RESET_ENCODER);
        driveFrontRight.setMode(RUN_WITHOUT_ENCODER);
        driveFrontRight.setZeroPowerBehavior(BRAKE);

        driveBackLeft = hardwareMap.get(DcMotor.class, "driveBackLeft");
        driveBackLeft.setMode(RUN_WITHOUT_ENCODER);
        driveBackLeft.setZeroPowerBehavior(BRAKE);
        driveBackLeft.setDirection(REVERSE);

        driveBackRight = hardwareMap.get(DcMotor.class, "driveBackRight");
        driveBackRight.setMode(RUN_WITHOUT_ENCODER);
        driveBackRight.setZeroPowerBehavior(BRAKE);

        intakeArm = hardwareMap.get(Servo.class, "intakeArm");

        outtakeSlide1 = hardwareMap.get(DcMotor.class, "outtakeSlide1");
        outtakeSlide1.setMode(STOP_AND_RESET_ENCODER);
        outtakeSlide1.setMode(RUN_USING_ENCODER);
        outtakeSlide1.setZeroPowerBehavior(BRAKE);
        outtakeSlide1.setDirection(REVERSE);

        outtakeSlide2 = hardwareMap.get(DcMotor.class, "outtakeSlide2");
        outtakeSlide2.setMode(STOP_AND_RESET_ENCODER);
        outtakeSlide2.setMode(RUN_USING_ENCODER);
        outtakeSlide2.setZeroPowerBehavior(BRAKE);
    }

}
