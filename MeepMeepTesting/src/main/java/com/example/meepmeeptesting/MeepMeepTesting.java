package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    public static void main(String[] args) {
        double MAX_VEL       =   52.48291908330528;
        double MAX_ACCEL     =   52.48291908330528;
        double MAX_ANG_VEL   =   Math.toRadians(222.7444266666667);
        double MAX_ANG_ACCEL =   Math.toRadians(222.7444266666667);
        double TRACK_WIDTH   =   13.5;
        double DELAY         =   1;
        MeepMeep meepMeep    =   new MeepMeep(650);

        Pose2d netZone = new Pose2d(55, 55, Math.toRadians(45));

        RoadRunnerBotEntity BlueRight = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-12, 65.5, Math.toRadians(-90)))
                        .splineTo(new Vector2d(0, 30), Math.toRadians(-90))
                        .waitSeconds(DELAY)
                        .back(0.1)
                        .splineTo(new Vector2d(-12, 40), Math.toRadians(-180))
                        .lineTo(new Vector2d(18,40))
                        .splineTo(new Vector2d(40,26), Math.toRadians(0))
                        .waitSeconds(DELAY)
                        .lineToLinearHeading(new Pose2d(55, 55, Math.toRadians(45)))
                        .waitSeconds(DELAY)
                        .lineToLinearHeading(new Pose2d(40, 26, Math.toRadians(0)))
                        .forward(10)
                        .waitSeconds(DELAY)
                        .lineToLinearHeading(new Pose2d(55, 55, Math.toRadians(45)))
                        .waitSeconds(DELAY)
                        .lineToLinearHeading(new Pose2d(50, 26, Math.toRadians(0)))
                        .forward(10)
                        .waitSeconds(DELAY)
                        .lineToLinearHeading(new Pose2d(55, 55, Math.toRadians(45)))
                        .waitSeconds(DELAY)
                        .lineToLinearHeading(new Pose2d(46, 38, Math.toRadians(180)))
                        .lineTo(new Vector2d(-24, 38))
                        .splineTo(new Vector2d(-40, 63), Math.toRadians(90))
                        .build());

        RoadRunnerBotEntity BlueLeft = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(12, 65.5, Math.toRadians(-90)))
                        .splineTo(new Vector2d(0, 30), Math.toRadians(-90))
                        .waitSeconds(DELAY)
                        .back(0.1)
                        .splineTo(new Vector2d(-12, 40), Math.toRadians(-180))
                        .lineTo(new Vector2d(18,40))
                        .splineTo(new Vector2d(40,26), Math.toRadians(0))
                        .waitSeconds(DELAY)
                        .lineToLinearHeading(new Pose2d(55, 55, Math.toRadians(45)))
                        .waitSeconds(DELAY)
                        .lineToLinearHeading(new Pose2d(40, 26, Math.toRadians(0)))
                        .forward(10)
                        .waitSeconds(DELAY)
                        .lineToLinearHeading(new Pose2d(55, 55, Math.toRadians(45)))
                        .waitSeconds(DELAY)
                        .lineToLinearHeading(new Pose2d(50, 26, Math.toRadians(0)))
                        .forward(10)
                        .waitSeconds(DELAY)
                        .lineToLinearHeading(new Pose2d(55, 55, Math.toRadians(45)))
                        .waitSeconds(DELAY)
                        .lineToLinearHeading(new Pose2d(46, 38, Math.toRadians(180)))
                        .lineTo(new Vector2d(-24, 38))
                        .splineTo(new Vector2d(-40, 63), Math.toRadians(90))
                        .build());

        RoadRunnerBotEntity RedRight = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-12, -65.5, Math.toRadians(90)))
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
                        .build());

        RoadRunnerBotEntity RedLeft = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(12, -65.5, Math.toRadians(90)))
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
                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_BLACK)
                .setDarkMode(true)
                .setBackgroundAlpha(1f)
                .addEntity(BlueRight)
                .addEntity(BlueLeft)
                .addEntity(RedRight)
                .addEntity(RedLeft)
                .start();
    }
}