
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.modules.YellowVisionPortal;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.Arrays;


@TeleOp(name="Sample Pipeline")
public class samplePipeline extends LinearOpMode {
    YellowVisionPortal yellowVisionPortal;
    private VisionPortal visionPortal;

    private void updatePosition() {
        yellowVisionPortal.position = Arrays.asList(0.0, 0.0, 0.0);
    }

    @Override
    public void runOpMode() {
        HardwareMap hwmap = hardwareMap;

        initOpenCV();
        waitForStart();

        while (opModeIsActive()) {
            updatePosition();
            telemetry.addData("Closest Sample [x,y, rotation]: ", Arrays.asList(yellowVisionPortal.centroid.x,yellowVisionPortal.centroid.y,yellowVisionPortal.angleOfRotation));
            telemetry.addData("time taken for image process: ",yellowVisionPortal.timeTakenMili);
            telemetry.update();


        }

        visionPortal.close();

    }


    private void initOpenCV() {
        yellowVisionPortal = new YellowVisionPortal();
        visionPortal = new VisionPortal.Builder().
                addProcessor(yellowVisionPortal)
                .setCameraResolution(new android.util.Size(1920, 1080))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam1"))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setAutoStartStreamOnBuild(true)
                .enableLiveView(true)
                .build();
    }







}
