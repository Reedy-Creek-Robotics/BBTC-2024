package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.modules.Robot;
import org.firstinspires.ftc.teamcode.modules.RunStates;
import org.firstinspires.ftc.teamcode.auto.BaseAuto;

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
                .splineTo(new Vector2d(55, 55), Math.toRadians(45))
               .build();

        Action spikeGrab1 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(47.5, 39), Math.toRadians(270))
                .build();

        Action spikeScore1 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(55, 55), Math.toRadians(225))
                .build();

        Action trajEnd = drive.actionBuilder(drive.localizer.getPose())

                .setReversed(false)
                .turnTo(Math.toRadians(180))
                .strafeTo(new Vector2d(-48, 60))
                .build();

        if (isStopRequested()) return;

        SequentialAction intakeSample = new SequentialAction(
                intakeSlide.intakeSlideOut(),
                intakeArm.intakeArmGrab(),
                intakeRotator.intakeRotatorGrab(),
                pincherRotator.pincherRotatorTurned(),
                pincher.pincherOpen(),
                new SleepAction(1),
                pincher.pincherClose(),
                new SleepAction(0.25),
                intakeSlide.intakeSlideIn(),
                intakeArm.intakeArmTransfer(),
                intakeRotator.intakeRotatorTransfer(),
                pincherRotator.pincherRotatorTurned(),
                new SleepAction(1),
                pincher.pincherOpen(),
                new SleepAction(0.25),
                intakeArm.intakeArmDefault(),
                intakeRotator.intakeRotatorDefault(),
                pincherRotator.pincherRotatorTurned()

        );

        SequentialAction outtakeSample = new SequentialAction(
                outtakeSlide.outtakeSlideUp(),
                basket.basketUp(),
                new SleepAction(0.75),
                basket.basketDown(),
                outtakeSlide.outtakeSlideDown());

        Actions.runBlocking(
                new SequentialAction(
                        preloadScore,
                        outtakeSample,
                        spikeGrab1,
                        intakeSample,
                        spikeScore1,
                        outtakeSample,
                        trajEnd

                )
        );
    }
}
