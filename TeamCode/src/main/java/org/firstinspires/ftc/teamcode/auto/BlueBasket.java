package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.components.*;

@Autonomous
public class BlueBasket extends LinearOpMode {

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

        waitForStart();

        Action preloadScore = drive.actionBuilder(initialPose).endTrajectory().fresh()
                .setReversed(true)
                .splineTo(new Vector2d(54, 54), Math.toRadians(45))
                .build();

        Action spikeGrab1 = drive.actionBuilder(drive.localizer.getPose())
                .setReversed(false)
                .splineTo(new Vector2d(48, 39), Math.toRadians(270))
                .build();

        Action spikeScore1 = drive.actionBuilder(drive.localizer.getPose())
                .setReversed(true)
                .splineTo(new Vector2d(54, 54), Math.toRadians(45))
                .build();

        Action spikeGrab2 = drive.actionBuilder(drive.localizer.getPose())
                .setReversed(false)
                .splineTo(new Vector2d(58, 39), Math.toRadians(270))
                .build();

        Action spikeScore2 = drive.actionBuilder(drive.localizer.getPose())
                .setReversed(true)
                .splineTo(new Vector2d(54, 54), Math.toRadians(45))
                .build();

        Action trajEnd = drive.actionBuilder(drive.localizer.getPose())

                .setReversed(false)
                .splineTo(new Vector2d(48, 24), Math.toRadians(270))
                .turn(Math.toRadians(180))
                .setReversed(true)
                .splineTo(new Vector2d(20, 10), Math.toRadians(180))
                .build();

        if (isStopRequested()) return;

        
        SequentialAction intakeSampleRegular = new SequentialAction(
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
                intakeArm.intakeArmDefault(),
                intakeRotator.intakeRotatorDefault(),
                pincherRotator.pincherRotatorTurned()
        );

        SequentialAction intakeSampleLast = new SequentialAction(
                intakeSlide.intakeSlideOut(),
                intakeArm.intakeArmGrab(),
                intakeRotator.intakeRotatorGrab(),
                pincherRotator.pincherRotatorLine(),
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
                intakeArm.intakeArmDefault(),
                intakeRotator.intakeRotatorDefault(),
                pincherRotator.pincherRotatorTurned()
        );

        SequentialAction outtakeSample = new SequentialAction(
                outtakeSlide.outtakeSlideUp(),
                drive.actionBuilder(drive.localizer.getPose())
                        .setReversed(true)
                        .splineTo(new Vector2d(55, 55), Math.toRadians(45))
                        .build(),
                basket.basketUp(),
                intakeSlide.intakeSlideOut(),
                intakeArm.intakeArmDefault(),
                intakeRotator.intakeRotatorPicking(),
                pincher.pincherOpen(),
                new SleepAction(0.75),
                basket.basketDown(),
                outtakeSlide.outtakeSlideDown());

        Actions.runBlocking(
                new SequentialAction(
                        preloadScore,
                        outtakeSample,
                        spikeGrab1,
                        intakeSampleRegular,
                        spikeScore1,
                        outtakeSample,
                        trajEnd

                )
        );
    }
}
