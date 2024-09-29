package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class ServoTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        telemetry.addLine("Game1 Up: Wrist");
        telemetry.addLine("Game1 Down: intakeGear");
        telemetry.update();

        while(opModeIsActive()) {
            if (gamepad1.dpad_up) {
                tester(0);
                break;
            } else if (gamepad1.dpad_down) {
                tester(1);
                break;
            }
        }
    }

    private void tester(int selection){
        ElapsedTime debounce = new ElapsedTime();
        Servo servo;
        if(selection == 0){
            servo = hardwareMap.get(Servo.class, "wrist");
        }else{
            servo = hardwareMap.get(Servo.class, "intakeGear");
        }

        double position = 0;

        while(opModeIsActive()) {
            telemetry.setNumDecimalPlaces(0, 1);
            telemetry.clearAll();
            telemetry.addData("Position", position);
            telemetry.addLine();
            telemetry.addLine("Dpad Up: Increase Position");
            telemetry.addLine("Dpad Down: Decrease Position");
            telemetry.update();

            if (gamepad1.dpad_up && debounce.milliseconds() > 200) {
                position += 0.1;
                debounce.reset();
            }

            if (gamepad1.dpad_down && debounce.milliseconds() > 200) {
                position -= 0.1;
                debounce.reset();
            }

            servo.setPosition(position);
        }
    }
}
