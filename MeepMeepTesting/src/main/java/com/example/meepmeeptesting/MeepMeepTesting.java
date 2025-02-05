package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.core.colorscheme.scheme.*;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    static double MAX_VEl = 60,
                MAX_ACCEL = 60,
              MAX_ANG_VEL = Math.PI,
            MAX_ANG_ACCEL = Math.PI,
              TRACK_WIDTH = 14.2117726773,
                    WIDTH = 17.75,
                   HEIGHT = 16;



    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700, 30);



        RoadRunnerBotEntity BlueBasket = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(MAX_VEl, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .setDimensions(WIDTH, HEIGHT)
                .setStartPose(new Pose2d(23-(WIDTH/2), 72-(HEIGHT/2), Math.toRadians(90)))
                .build();

        BlueBasket.runAction(BlueBasket.getDrive().actionBuilder(new Pose2d(23-(WIDTH/2), 72-(HEIGHT/2), Math.toRadians(90)))

                // <preloadScore>
                .setReversed(true)
                .splineTo(new Vector2d(54, 54), Math.toRadians(45))
                // </preloadScore>

                .waitSeconds(1)



                .waitSeconds(1)

                /*// <spikeGrab1>
                .setReversed(false)
                .splineTo(new Vector2d(48, 39), Math.toRadians(270))
                // </spikeGrab1>

                .waitSeconds(1)

                // <spikeScore1>
                .setReversed(true)
                .splineTo(new Vector2d(54, 54), Math.toRadians(45))
                // </spikeScore1>

                .waitSeconds(1)

                // <outtakeSample>
                .setReversed(true)
                .splineTo(new Vector2d(55, 55), Math.toRadians(45))
                // </outtakeSample>

                .waitSeconds(1)

                // <spikeGrab2>
                .setReversed(false)
                .splineTo(new Vector2d(58, 39), Math.toRadians(270))
                // </spikeGrab2>

                .waitSeconds(1)

                // <spikeScore2>
                .setReversed(true)
                .splineTo(new Vector2d(54, 54), Math.toRadians(45))
                // </spikeScore2>

                .waitSeconds(1)

                // <outtakeSample>
                .setReversed(true)
                .splineTo(new Vector2d(55, 55), Math.toRadians(45))
                // </outtakeSample>

                .waitSeconds(1)

                .waitSeconds(1)*/

                .build());

        BlueBasket.runAction(BlueBasket.getDrive().actionBuilder(BlueBasket.getPose())
                // <outtakeSample>
                .setReversed(true)
                .splineTo(new Vector2d(55, 55), Math.toRadians(45))
                // </outtakeSample>
                .build());

        BlueBasket.runAction(BlueBasket.getDrive().actionBuilder(BlueBasket.getPose())
                // <trajEnd>
                .setReversed(false)
                .splineTo(new Vector2d(48, 24), Math.toRadians(270))
                .turn(Math.toRadians(180))
                .setReversed(true)
                .splineTo(new Vector2d(20, 10), Math.toRadians(180))
                // </trajEnd>
                .build());

        RoadRunnerBotEntity BlueObservation = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(MAX_VEl, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .setDimensions(WIDTH, HEIGHT)
                .setStartPose(new Pose2d(-24+(WIDTH/2), 72-(HEIGHT/2), Math.toRadians(90)))
                .build();



        RoadRunnerBotEntity RedBasket = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(MAX_VEl, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .setDimensions(WIDTH, HEIGHT)
                .setStartPose(new Pose2d(-14.125, -64, Math.toRadians(270)))
                .build();

        RedBasket.runAction(RedBasket.getDrive().actionBuilder(new Pose2d(-14.125, -64, Math.toRadians(270)))

                .setReversed(true)
                .splineTo(new Vector2d(-54, -54), Math.toRadians(225))

                        .waitSeconds(1)

                .setReversed(true)
                .splineTo(new Vector2d(-56, -56), Math.toRadians(225))

                .waitSeconds(1)

                .splineTo(new Vector2d(-50, -50), Math.toRadians(45))

                        .waitSeconds(1)

                .setReversed(false)
                .splineTo(new Vector2d(-46, -44), Math.toRadians(90))

                .build());

        RoadRunnerBotEntity RedObservation = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(MAX_VEl, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .setDimensions(WIDTH, HEIGHT)
                .setStartPose(new Pose2d(24-(WIDTH/2), -72+(HEIGHT/2), Math.toRadians(270)))
                .build();

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(false)
                .setAxesInterval(24)
                .setBackgroundAlpha(0.95f)
                //.addEntity(BlueBasket)
                //.addEntity(BlueObservation)
                .addEntity(RedBasket)
                //.addEntity(RedObservation)
                .start();
    }
}