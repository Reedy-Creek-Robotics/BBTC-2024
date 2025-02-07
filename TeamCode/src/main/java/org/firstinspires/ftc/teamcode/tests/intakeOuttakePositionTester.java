package org.firstinspires.ftc.teamcode.tests;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp(group="test")
public class intakeOuttakePositionTester extends LinearOpMode {

    static int buttonDelay = 60;
    int outtakeSlidePosition = 0;

    double intakeSlidePosition = 0.73;
    double intakeArmPosition = 0.6;
    double intakeRotatorPosition = 0.02;
    double pincherRotatorPosition = 0.65;
    double pincherPosition = 0.5;
    double basketPosition = 0.2;

    Servo intakeSlide;
    Servo intakeArm;
    Servo intakeRotator;
    Servo pincherRotator;
    Servo pincher;
    Servo basket;
    
    DcMotor outtakeSlideLeft;
    DcMotor outtakeSlideRight;

    ElapsedTime debounceTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    public void runOpMode() throws InterruptedException {

        intakeSlide = hardwareMap.get(Servo.class, "intakeSlide");
        intakeArm = hardwareMap.get(Servo.class, "intakeArm");
        intakeRotator = hardwareMap.get(Servo.class, "intakeRotator");
        pincherRotator = hardwareMap.get(Servo.class, "pincherRotator");
        pincher = hardwareMap.get(Servo.class, "pincher");
        basket = hardwareMap.get(Servo.class, "basket");
        outtakeSlideLeft = hardwareMap.get(DcMotor.class, "outtakeSlideLeft");
        outtakeSlideRight = hardwareMap.get(DcMotor.class, "outtakeSlideRight");
        outtakeSlideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeSlideLeft.setZeroPowerBehavior(BRAKE);
        outtakeSlideRight.setZeroPowerBehavior(BRAKE);

        waitForStart();
        while (opModeIsActive()){
               if (gamepad1.a && debounceTimer.milliseconds() > buttonDelay){
                    intakeSlidePosition += 0.02;
                    debounceTimer.reset();
               }
               if (gamepad1.b && debounceTimer.milliseconds() > buttonDelay){
                    intakeSlidePosition -= 0.02;
                    debounceTimer.reset();
               }
               if (gamepad1.x && debounceTimer.milliseconds() > buttonDelay){
                    intakeArmPosition += 0.02;
                    debounceTimer.reset();
               }
               if (gamepad1.y && debounceTimer.milliseconds() > buttonDelay){
                    intakeArmPosition -= 0.02;
                    debounceTimer.reset();
               }
               if (gamepad1.dpad_up && debounceTimer.milliseconds() > buttonDelay){
                    intakeRotatorPosition += 0.02;
                    debounceTimer.reset();
               }
               if (gamepad1.dpad_down && debounceTimer.milliseconds() > buttonDelay){
                    intakeRotatorPosition -= 0.02;
                    debounceTimer.reset();
               }
               if (gamepad1.dpad_left && debounceTimer.milliseconds() > buttonDelay){
                    pincherRotatorPosition += 0.02;
                    debounceTimer.reset();
               }
               if (gamepad1.dpad_right && debounceTimer.milliseconds() > buttonDelay){
                    pincherRotatorPosition -= 0.02;
                    debounceTimer.reset();
               }
               if (gamepad1.left_bumper && debounceTimer.milliseconds() > buttonDelay){
                    pincherPosition += 0.02;
                    debounceTimer.reset();
               }
               if (gamepad1.right_bumper && debounceTimer.milliseconds() > buttonDelay){
                    pincherPosition -= 0.02;
                    debounceTimer.reset();
               }
               if (gamepad1.left_stick_button && debounceTimer.milliseconds() > buttonDelay){
                    basketPosition += 0.02;
                    debounceTimer.reset();
               }
               if (gamepad1.right_stick_button && debounceTimer.milliseconds() > buttonDelay){
                    basketPosition -= 0.02;
                    debounceTimer.reset();
               }

               telemetry.addData("Intake Slide Position", intakeSlidePosition);
               telemetry.addData("Intake Arm Position", intakeArmPosition);
               telemetry.addData("Intake Rotator Position", intakeRotatorPosition);
               telemetry.addData("Pincher Rotator Position", pincherRotatorPosition);
               telemetry.addData("Pincher Position", pincherPosition);
               telemetry.addData("Basket Position", basketPosition);
               telemetry.addData("Outtake Slide Position", (outtakeSlideRight.getCurrentPosition() + outtakeSlideLeft.getCurrentPosition())/2);
               telemetry.update();


               outtakeSlideLeft.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
               outtakeSlideRight.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
               intakeArm.setPosition(intakeArmPosition);
               intakeSlide.setPosition(intakeSlidePosition);
               intakeRotator.setPosition(intakeRotatorPosition);
               pincherRotator.setPosition(pincherRotatorPosition);
               pincher.setPosition(pincherPosition);
               basket.setPosition(basketPosition);
               
        }
    }
}
