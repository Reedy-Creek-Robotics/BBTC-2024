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
              MAX_ANG_VEL = Math.toRadians(180),
            MAX_ANG_ACCEL = Math.toRadians(180),
              TRACK_WIDTH = 15,
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
                .setReversed(true)
                //.splineTo(new Vector2d(48, 48), Math.toRadians(45))
                .splineTo(new Vector2d(58, 58), Math.toRadians(45))
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
                .setStartPose(new Pose2d(-24+(WIDTH/2), -72+(HEIGHT/2), Math.toRadians(270)))
                .build();



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
                .addEntity(BlueBasket)
                //.addEntity(BlueObservation)
                //.addEntity(RedBasket)
                //.addEntity(RedObservation)
                .start();
    }
}