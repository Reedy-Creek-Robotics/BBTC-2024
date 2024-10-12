package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class RedRightSelfish extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        double DELAY = 1;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence path = drive.trajectorySequenceBuilder(new Pose2d(-12, 65.5, Math.toRadians(-90)))
                .splineTo(new Vector2d(0, -30), Math.toRadians(90))
                .waitSeconds(DELAY)
                .back(0.1)
                .splineTo(new Vector2d(12, -40), Math.toRadians(0))
                .lineTo(new Vector2d(-18,-40))
                .splineTo(new Vector2d(-40,-26), Math.toRadians(180))
                .waitSeconds(DELAY)
                .lineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(-135)))
                .waitSeconds(DELAY)
                .lineToLinearHeading(new Pose2d(-40, -26, Math.toRadians(180)))
                .forward(10)
                .waitSeconds(DELAY)
                .lineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(-135)))
                .waitSeconds(DELAY)
                .lineToLinearHeading(new Pose2d(-50, -26, Math.toRadians(180)))
                .forward(10)
                .waitSeconds(DELAY)
                .lineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(-135)))
                .waitSeconds(DELAY)
                .lineToLinearHeading(new Pose2d(-46, -38, Math.toRadians(0)))
                .lineTo(new Vector2d(24, -38))
                .splineTo(new Vector2d(40, -63), Math.toRadians(-90))
                .build();

        waitForStart();

        drive.followTrajectorySequence(path);
    }
}
