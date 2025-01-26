package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class parkOnly extends LinearOpMode {

    DcMotor driveFrontLeft;
    DcMotor driveFrontRight;
    DcMotor driveBackLeft;
    DcMotor driveBackRight;

    ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {
        driveFrontLeft = hardwareMap.get(DcMotor.class, "driveFrontLeft");
        driveFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        driveFrontRight = hardwareMap.get(DcMotor.class, "driveFrontRight");
        driveBackLeft = hardwareMap.get(DcMotor.class, "driveBackLeft");
        driveBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        driveBackRight = hardwareMap.get(DcMotor.class, "driveBackRight");

        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        waitForStart();

        driveFrontLeft.setPower(0.5);
        driveFrontRight.setPower(-0.5);
        driveBackLeft.setPower(-0.5);
        driveBackRight.setPower(0.5);
        timer.reset();

        while(timer.seconds() < 2 && opModeIsActive()){
            telemetry.addData("Moving for", timer.milliseconds());
        }

        driveFrontLeft.setPower(0);
        driveFrontRight.setPower(0);
        driveBackLeft.setPower(0);
        driveBackRight.setPower(0);



    }
}
