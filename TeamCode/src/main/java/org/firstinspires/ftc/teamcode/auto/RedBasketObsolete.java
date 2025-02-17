package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.components.*;

@Disabled
@Autonomous(preselectTeleOp = "Tele-Op NO CAM")
public class RedBasketObsolete extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-14.125, -64, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        OuttakeSlide outtakeSlide = new OuttakeSlide(hardwareMap);
        Basket basket = new Basket(hardwareMap);
        Pincher pincher = new Pincher(hardwareMap);
        //Claw claw = new Claw(hardwareMap);
        IntakeSlide intakeSlide = new IntakeSlide(hardwareMap);
        IntakeArm intakeArm = new IntakeArm(hardwareMap);
        PincherRotator pincherRotator = new PincherRotator(hardwareMap);
        IntakeRotator intakeRotator = new IntakeRotator(hardwareMap);

        Actions.runBlocking(basket.basketDown());

        Action preloadScore = drive.actionBuilder(initialPose).endTrajectory().fresh()
                .setReversed(true)
                .splineTo(new Vector2d(-54, -54), Math.toRadians(225))
                .build();

        ParallelAction spikeGrab1 = new ParallelAction(
                drive.actionBuilder(new Pose2d(-47, -47, Math.toRadians(45))).fresh()
                        .setReversed(false)
                        .lineToX(-45.5)
                        .turnTo(Math.toRadians(90))
                        .lineToY(-44.5)
                        .build(),
                outtakeSlide.outtakeSlideDown());

        Action spikeScore1 = drive.actionBuilder(new Pose2d(-47, -39, Math.toRadians(90))).fresh()
                .setReversed(true)
                .splineTo(new Vector2d(-54, -54), Math.toRadians(225))
                .build();



        ParallelAction spikeGrab2 = new ParallelAction(
                drive.actionBuilder(new Pose2d(-52.5, -50, Math.toRadians(45))).fresh()
                        .setReversed(false)
                        //.strafeToSplineHeading(new Vector2d(-44, -45), Math.toRadians(90))
                        .strafeToSplineHeading(new Vector2d(-61, -43), Math.toRadians(90))
                        .build(),
                outtakeSlide.outtakeSlideDown());



        Action spikeScore2 = drive.actionBuilder(new Pose2d(-54, -39, Math.toRadians(90))).fresh()
                .setReversed(true)
                .splineTo(new Vector2d(-54, -54), Math.toRadians(230))
                .build();



        ParallelAction trajEnd = new ParallelAction(
                drive.actionBuilder(new Pose2d(-50, -50, Math.toRadians(45))).fresh()
                        .setReversed(false)
                        .splineTo(new Vector2d(-48, -24), Math.toRadians(90))
                        .turn(Math.toRadians(180))
                        .setReversed(true)
                        .splineTo(new Vector2d(-20, -10), Math.toRadians(0))
                        .build(),
                outtakeSlide.outtakeSlideDown()
        );

        if (isStopRequested()) return;

        SequentialAction endServoPositions = new SequentialAction(
                new SleepAction(1),
                new ParallelAction(
                        intakeSlide.intakeSlideIn(),
                        intakeRotator.intakeRotatorDefault(),
                        intakeArm.intakeArmDefault(),
                        outtakeSlide.outtakeSlideDown()
                ));

        SequentialAction intakeSample1 = new SequentialAction(
                intakeSlide.intakeSlideOut(),
                intakeArm.intakeArmGrab(),
                intakeRotator.intakeRotatorGrab(),
                pincherRotator.pincherRotatorTurned(),
                pincher.pincherOpen(),
                new SleepAction(1),
                pincher.pincherClose(),
                new SleepAction(0.5),
                intakeSlide.intakeSlideIn(),
                intakeArm.intakeArmTransfer(),
                intakeRotator.intakeRotatorTransfer(),
                pincherRotator.pincherRotatorTurned(),
                new SleepAction(1.5),
                pincher.pincherOpen(),
                new SleepAction(0.5),
                intakeSlide.intakeSlideOut(),
                new SleepAction(0.5),
                intakeArm.intakeArmDefault(),
                intakeRotator.intakeRotatorDefault(),
                pincherRotator.pincherRotatorTurned()
        );

        SequentialAction intakeSample2 = new SequentialAction(
                intakeSlide.intakeSlideOut(),
                intakeArm.intakeArmGrab(),
                intakeRotator.intakeRotatorGrab(),
                pincherRotator.pincherRotatorTurned(),
                pincher.pincherOpen(),
                new SleepAction(0.5),
                pincher.pincherClose(),
                new SleepAction(0.5),
                intakeSlide.intakeSlideIn(),
                intakeArm.intakeArmTransfer(),
                intakeRotator.intakeRotatorTransfer(),
                pincherRotator.pincherRotatorTurned(),
                new SleepAction(1.5),
                pincher.pincherOpen(),
                new SleepAction(0.5),
                intakeSlide.intakeSlideOut(),
                new SleepAction(0.5),
                intakeArm.intakeArmDefault(),
                intakeRotator.intakeRotatorDefault(),
                pincherRotator.pincherRotatorTurned()
        );



        SequentialAction outtakeSample1 = new SequentialAction(
                outtakeSlide.outtakeSlideUp(),
                drive.actionBuilder(new Pose2d(-54, -54, Math.toRadians(45)))
                        .setReversed(true)
                        .splineTo(new Vector2d(-56, -56), Math.toRadians(225))
                        .build(),
                basket.basketUp(),
                intakeArm.intakeArmDefault(),
                intakeRotator.intakeRotatorPicking(),
                pincher.pincherOpen(),
                new SleepAction(0.75),
                drive.actionBuilder(new Pose2d(-55, -55, Math.toRadians(45)))
                        .splineTo(new Vector2d(-50, -50), Math.toRadians(45))
                        .build(),
                basket.basketDown());



        SequentialAction outtakeSample2 = new SequentialAction(
                outtakeSlide.outtakeSlideUp(),
                drive.actionBuilder(new Pose2d(-54, -54, Math.toRadians(45)))
                        .setReversed(true)
                        .splineTo(new Vector2d(-56, -56), Math.toRadians(225))
                        .build(),
                basket.basketUp(),
                intakeSlide.intakeSlideOut(),
                intakeArm.intakeArmDefault(),
                intakeRotator.intakeRotatorPicking(),
                pincher.pincherOpen(),
                new SleepAction(0.75),

                drive.actionBuilder(new Pose2d(-55, -55, Math.toRadians(45)))
                        .splineTo(new Vector2d(-50, -50), Math.toRadians(45)).build(),
                basket.basketDown());



        SequentialAction outtakeSample3 = new SequentialAction(
                outtakeSlide.outtakeSlideUp(),
                drive.actionBuilder(new Pose2d(-54, -54, Math.toRadians(45)))
                        .setReversed(true)
                        .splineTo(new Vector2d(-56, -56), Math.toRadians(225))
                        .build(),
                basket.basketUp(),
                intakeSlide.intakeSlideOut(),
                intakeArm.intakeArmDefault(),
                intakeRotator.intakeRotatorPicking(),
                pincher.pincherOpen(),
                new SleepAction(0.75),
                drive.actionBuilder(new Pose2d(-55, -55, Math.toRadians(45)))
                        .splineTo(new Vector2d(-50, -50), Math.toRadians(45))
                        .build(),
                basket.basketDown());



        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        preloadScore,
                        outtakeSample1,
                        spikeGrab1,
                        intakeSample1,
                        spikeScore1,
                        outtakeSample2,
                        spikeGrab2,
                        intakeSample2,
                        spikeScore2,
                        outtakeSample3,
                        endServoPositions/*,
                        trajEnd*/
                )
        );
    }
}
