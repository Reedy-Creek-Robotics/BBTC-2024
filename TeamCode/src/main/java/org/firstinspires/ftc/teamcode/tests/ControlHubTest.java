package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ControlHubTest extends LinearOpMode {

    DcMotor mPort0, mPort1, mPort2, mPort3;
    Servo sPort0, sPort1, sPort2, sPort3, sPort4, sPort5;

    @Override
    public void runOpMode() throws InterruptedException {
        mPort0 = hardwareMap.get(DcMotor.class, "mPort0");
        mPort1 = hardwareMap.get(DcMotor.class, "mPort1");
        mPort2 = hardwareMap.get(DcMotor.class, "mPort2");
        mPort3 = hardwareMap.get(DcMotor.class, "mPort3");

        sPort0 = hardwareMap.get(Servo.class, "sPort0");
        sPort1 = hardwareMap.get(Servo.class, "sPort1");
        sPort2 = hardwareMap.get(Servo.class, "sPort2");
        sPort3 = hardwareMap.get(Servo.class, "sPort3");
        sPort4 = hardwareMap.get(Servo.class, "sPort4");
        sPort5 = hardwareMap.get(Servo.class, "sPort5");

        telemetry.addLine("Press up for motor, down for servo");
        telemetry.update();

        if(gamepad1.dpad_up){
            // Motor Testing
            while(gamepad1.dpad_up);
            int curMotor = 0;
            telemetry.addData("Current Motor", curMotor);

        }else if(gamepad1.dpad_down){
            // Servo Testing

        }
    }
}
