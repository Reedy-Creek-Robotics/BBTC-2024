package org.firstinspires.ftc.teamcode.tests;

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
import org.openftc.easyopencv.OpenCvCamera;

@TeleOp(name = "TEST Tele-Op")
public class TestTeleOp extends LinearOpMode {

    ElapsedTime buttonDebounce;

    double ly1;
    double lx1;
    double rx1;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    DcMotor driveFrontLeft;
    DcMotor driveFrontRight;
    DcMotor driveBackLeft;
    DcMotor driveBackRight;
    DcMotor outtakeSlideRight;
    DcMotor outtakeSlideLeft;

    @Override
    public void runOpMode() throws InterruptedException {

        initHardware();

        buttonDebounce = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        telemetry.addLine("> PRESS START");
        waitForStart();

        while (opModeIsActive()) {

            processVariableUpdates();

            processDriving();
            processControl();
            processTelemetry();

        }
    }

    private void processDriving() {

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
        double slidePower = gamepad1.right_trigger - gamepad1.left_trigger;

        outtakeSlideRight.setPower(slidePower);
        outtakeSlideLeft.setPower(slidePower);
    }

    private void processVariableUpdates() {
        ly1 = -gamepad1.left_stick_y;
        // We apply a 1.1 multiplier to the x-axis to make apply for strafing inaccuracies
        lx1 = gamepad1.left_stick_x * 1.1;
        rx1 = gamepad1.right_stick_x;

        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);
    }

    private void processTelemetry(){
        telemetry.addData("Outtake Slide Position", (outtakeSlideRight.getCurrentPosition() + outtakeSlideLeft.getCurrentPosition()) / 2);
        telemetry.update();
    }

    private void initHardware(){
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

        outtakeSlideLeft = hardwareMap.get(DcMotor.class, "outtakeSlideLeft");
        outtakeSlideLeft.setMode(STOP_AND_RESET_ENCODER);
        outtakeSlideLeft.setMode(RUN_USING_ENCODER);
        outtakeSlideLeft.setZeroPowerBehavior(BRAKE);
        outtakeSlideLeft.setDirection(REVERSE);
    }
}