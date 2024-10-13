package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class RedRightChamberOnly extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence path = drive.trajectorySequenceBuilder(new Pose2d(-12, 65.5, Math.toRadians(-90)))
                .waitSeconds(10)
                .splineTo(new Vector2d(-8, -30), Math.toRadians(90))
                .waitSeconds(3)
                .back(12)
                .splineTo(new Vector2d(60, -60), Math.toRadians(0))
                .build();

        waitForStart();

        drive.followTrajectorySequence(path);
    }
}