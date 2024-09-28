package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TeleOpDrive extends LinearOpMode {

    ElapsedTime speedFactorDebounce;
    ElapsedTime intakeSpeedFactorDebounce;

    double speedFactor = 0.7;
    double intakeSpeedFactor = 0.5;
    double ly1;
    double lx1;
    double rx1;
    double lt2;
    double rt2;
    double lt1;
    double rt1;

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

    public void runOpMode() throws InterruptedException {
        waitForStart();

        processVariableUpdates();
        processDriving();
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

}
