package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ParkOnly extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor driveFrontLeft = hardwareMap.get(DcMotor.class, "driveFrontLeft");
        DcMotor driveFrontRight = hardwareMap.get(DcMotor.class, "driveFrontRight");
        DcMotor driveBackLeft = hardwareMap.get(DcMotor.class, "driveBackLeft");
        DcMotor driveBackRight = hardwareMap.get(DcMotor.class, "driveBackRight");


    }
}
