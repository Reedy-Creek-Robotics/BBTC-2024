package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
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
public class TeleOpDrive extends LinearOpMode {

    ElapsedTime speedFactorDebounce;
    ElapsedTime intakeSpeedFactorDebounce;
    ElapsedTime wristPositionDebounce;

    double wristPosition = 0.5;
    double speedFactor = 0.7;
    double intakeSpeedFactor = 0.5;
    double ly1;
    double lx1;
    double rx1;
    double lt2;
    double rt2;
    double lt1;
    double rt1;
    double ly2;

    Servo intakeGear;
    Servo wrist;

    DcMotor driveFrontLeft;
    DcMotor driveFrontRight;
    DcMotor driveBackLeft;
    DcMotor driveBackRight;
    DcMotor intakeSlide1;
    DcMotor intakeSlide2;
    DcMotor intakeArm;

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
        lt2 = gamepad2.left_trigger;
        rt2 = gamepad2.right_trigger;
        lt1 = gamepad1.left_trigger;
        rt1 = gamepad1.right_trigger;
        ly2 = gamepad2.left_stick_y;

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
        if(gamepad2.dpad_up && wristPositionDebounce.milliseconds() <= buttonDelay){
            wristPosition += 0.1;
            wristPositionDebounce.reset();
        }
        if(gamepad2.dpad_down && wristPositionDebounce.milliseconds() <= buttonDelay){
            wristPosition -= 0.1;
            wristPositionDebounce.reset();
        }
    }

    private void processControl(){
        double intakeSlidePower = rt1-lt1;
        double intakeArmPower = ly2;

        intakeSlide1.setPower(intakeSlidePower * intakeSpeedFactor);
        intakeSlide2.setPower(intakeSlidePower * intakeSpeedFactor);
        intakeArm.setPower(intakeArmPower * intakeSpeedFactor);

        wrist.setPosition(wristPosition);

        if(gamepad2.left_bumper){
            intakeGear.setPosition(0.4);
        }
        if(gamepad2.right_bumper){
            intakeGear.setPosition(1);
        }

    }
    private void processTelemetry(){


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

        intakeArm = hardwareMap.get(DcMotor.class, "intakeArm");
        intakeArm.setMode(STOP_AND_RESET_ENCODER);
        intakeArm.setMode(RUN_USING_ENCODER);
        intakeArm.setZeroPowerBehavior(BRAKE);

        intakeSlide1 = hardwareMap.get(DcMotor.class, "intakeSlide1");
        intakeSlide1.setMode(STOP_AND_RESET_ENCODER);
        intakeSlide1.setMode(RUN_USING_ENCODER);
        intakeSlide1.setZeroPowerBehavior(BRAKE);
        intakeSlide1.setDirection(REVERSE);

        intakeSlide2 = hardwareMap.get(DcMotor.class, "intakeSlide2");
        intakeSlide2.setMode(STOP_AND_RESET_ENCODER);
        intakeSlide2.setMode(RUN_USING_ENCODER);
        intakeSlide2.setZeroPowerBehavior(BRAKE);

        wrist = hardwareMap.get(Servo.class, "wrist");
        intakeGear = hardwareMap.get(Servo.class, "intakeGear");
    }

}
