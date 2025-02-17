package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.components.*;

@Autonomous(preselectTeleOp = "Tele-Op NO CAM")
public class BlueBasket extends LinearOpMode {

    Vector2d spike1 = new Vector2d(48, 26);
    Vector2d spike2 = new Vector2d(58, 26);
    Vector2d spike3 = new Vector2d(68, 26);

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(14.125, 64, Math.toRadians(90));
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

        ParallelAction preloadScore = new ParallelAction(
                drive.actionBuilder(initialPose).endTrajectory()
                        .setReversed(true)
                        .splineTo(new Vector2d(57, 57), Math.toRadians(65))
                        .build(),
                outtakeSlide.outtakeSlideUp()
        );

        SequentialAction outtakeSample1 = new SequentialAction(
                outtakeSlide.outtakeSlideUp(),
                basket.basketUp(),
                intakeSlide.intakeSlideOut(),
                intakeArm.intakeArmDefault(),
                pincherRotator.pincherRotatorTurned(),
                intakeRotator.intakeRotatorPicking(),
                pincher.pincherOpen(),
                new SleepAction(0.75),
                basket.basketDown()
        );

        ParallelAction spikeGrab1 = new ParallelAction(
                new SequentialAction(
                        drive.actionBuilder(new Pose2d(57, 57, Math.toRadians(245)))
                                .setReversed(false)
                                .strafeToSplineHeading(new Vector2d(45, spike1.y+19), Math.toRadians(270))
                                .build()),
                new SequentialAction(
                        new SleepAction(0.4),
                        outtakeSlide.outtakeSlideDown()
                )
        );

        SequentialAction intakeSample1 = new SequentialAction(
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

        ParallelAction spikeScore1 = new ParallelAction(
                drive.actionBuilder(new Pose2d(45, spike1.y+19, Math.toRadians(270)))
                        .setReversed(true)
                        .splineTo(new Vector2d(54, 54), Math.toRadians(45))
                        .build(),
                outtakeSlide.outtakeSlideUp()
        );

        SequentialAction outtakeSample2 = new SequentialAction(
                outtakeSlide.outtakeSlideUp(),
                drive.actionBuilder(new Pose2d(54, 54, Math.toRadians(225)))
                        .setReversed(true)
                        .strafeToSplineHeading(new Vector2d(57, 57), Math.toRadians(225))
                        .build(),
                basket.basketUp(),
                intakeSlide.intakeSlideOut(),
                intakeArm.intakeArmDefault(),
                pincherRotator.pincherRotatorTurned(),
                intakeRotator.intakeRotatorPicking(),
                pincher.pincherOpen(),
                new SleepAction(0.75),
                basket.basketDown()
        );

        ParallelAction spikeGrab2 = new ParallelAction(
                drive.actionBuilder(new Pose2d(57, 57, Math.toRadians(225)))
                        .setReversed(false)
                        .strafeToSplineHeading(new Vector2d(56, spike2.y+20.5), Math.toRadians(270))
                        .build(),
                new SequentialAction(
                        new SleepAction(0.4),
                        outtakeSlide.outtakeSlideDown()
                )
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
                intakeArm.intakeArmDefault(),
                intakeRotator.intakeRotatorDefault(),
                pincherRotator.pincherRotatorTurned()
        );

        ParallelAction spikeScore2 = new ParallelAction(
                drive.actionBuilder(new Pose2d(56, spike2.y+20.5, Math.toRadians(270)))
                        .setReversed(true)
                        .splineTo(new Vector2d(54, 54), Math.toRadians(45))
                        .build(),
                outtakeSlide.outtakeSlideUp()
        );

        SequentialAction outtakeSample3 = new SequentialAction(
                outtakeSlide.outtakeSlideUp(),
                drive.actionBuilder(new Pose2d(54, 54, Math.toRadians(225)))
                        .setReversed(true)
                        .strafeToSplineHeading(new Vector2d(57, 57), Math.toRadians(225))
                        .build(),
                basket.basketUp(),
                intakeSlide.intakeSlideOut(),
                intakeArm.intakeArmDefault(),
                pincherRotator.pincherRotatorLine(),
                intakeRotator.intakeRotatorPicking(),
                pincher.pincherOpen(),
                new SleepAction(0.75),
                basket.basketDown()
        );

        ParallelAction spikeGrab3 = new ParallelAction(
                drive.actionBuilder(new Pose2d(57, 57, Math.toRadians(225)))
                        .setReversed(false)
                        .splineTo(new Vector2d(40, 26), Math.toRadians(270))
                        .turnTo(Math.toRadians(0))
                        .strafeToSplineHeading(new Vector2d(spike3.x+10, 26), Math.toRadians(0))
                        .build(),
                new SequentialAction(
                        new SleepAction(0.4),
                        outtakeSlide.outtakeSlideDown()
                ));

        SequentialAction intakeSample3 = new SequentialAction(
                intakeSlide.intakeSlideOut(),
                intakeArm.intakeArmGrab(),
                intakeRotator.intakeRotatorGrab(),
                pincherRotator.pincherRotatorLine(),
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

        ParallelAction spikeScore3 = new ParallelAction(
                drive.actionBuilder(new Pose2d(spike3.x-19, 26, Math.toRadians(270)))
                        .setReversed(true)
                        .splineTo(new Vector2d(54, 54), Math.toRadians(45))
                        .build(),
                outtakeSlide.outtakeSlideUp()
        );

        SequentialAction outtakeSample4 = new SequentialAction(
                outtakeSlide.outtakeSlideUp(),
                drive.actionBuilder(new Pose2d(54, 54, Math.toRadians(225)))
                        .setReversed(true)
                        .splineTo(new Vector2d(56, 56), Math.toRadians(235))
                        .build(),
                basket.basketUp(),
                intakeSlide.intakeSlideOut(),
                intakeArm.intakeArmDefault(),
                pincherRotator.pincherRotatorTurned(),
                intakeRotator.intakeRotatorPicking(),
                pincher.pincherOpen(),
                new SleepAction(0.75),
                basket.basketDown()
        );

        SequentialAction endPositions = new SequentialAction(
                drive.actionBuilder(new Pose2d(56, 56, Math.toRadians(235)))
                        .splineTo(new Vector2d(52, 52), Math.toRadians(235))
                        .build(),
                intakeSlide.intakeSlideIn(),
                intakeRotator.intakeRotatorDefault(),
                intakeArm.intakeArmDefault()

        );

        ParallelAction levelOnePark = new ParallelAction(
                new SequentialAction(
                        new SleepAction(0.2),
                        outtakeSlide.outtakeSlidePark()
                ),
                drive.actionBuilder(new Pose2d(52, 52, Math.toRadians(235)))
                        .splineTo(new Vector2d(48, 26), Math.toRadians(270))
                        .turnTo(Math.toRadians(90))
                        .setReversed(true)
                        .splineTo(new Vector2d(26, 10), Math.toRadians(180))
                        .build()
        );

        telemetry.addLine("READY");
        telemetry.update();

        if (isStopRequested()) return;

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
                        /*spikeGrab3,
                        intakeSample3,
                        spikeScore3,
                        outtakeSample4,*/
                        endPositions,
                        levelOnePark
                )
        );
    }
}