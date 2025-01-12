package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;


@TeleOp(name="OpenCV testing")
public class OpenCVTest extends LinearOpMode {
    final int width = 1920;
    final int height = 1080;
    private OpenCvCamera controlHubCam;
    private Point centroid = new Point();
    private  double angleOfRotation = 0;
    private List<Double> position = Arrays.asList(0.0, 0.0, 0.0);

    private double timeTakenMili;
    private void updatePosition() {
        position = Arrays.asList(0.0, 0.0, 0.0);
    }

    @Override
    public void runOpMode() {
        HardwareMap hwmap = hardwareMap;
        initOpenCV();

        waitForStart();
        Gamepad gamepad1 = new Gamepad();


        while (opModeIsActive()) {
            updatePosition();
            if (gamepad1.x) {
                telemetry.addData("Closest Sample [x,y, rotation]: ", Arrays.asList(centroid.x,centroid.y,angleOfRotation));
                telemetry.addData("time taken for image process: ", timeTakenMili);

            }
        }
        controlHubCam.stopStreaming();
    }


    private void initOpenCV() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);


        controlHubCam.setPipeline(new YellowSampleDetection());

        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);


    }
    class YellowSampleDetection extends OpenCvPipeline{
        final double width = 1920;
        final double height = 1080;
        final double screenCenterX = width/2;
        final double screenCenterY = height/2;
        final double distanceOffGround = 10.5;
        @Override
        public Mat processFrame(Mat input){
            long startTime = System.nanoTime();
            List<List<Object>> samplesData = new ArrayList<>();

            Mat mask = preprocess(input);
            ArrayList<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            Imgproc.drawContours(input, contours, -1, new Scalar(0,0,255));

            for(MatOfPoint c: contours){
                if(Imgproc.contourArea(c)>15000){
                    MatOfPoint2f c2f = new MatOfPoint2f(c.toArray());
                    double epsilon = 0.0129032258 * Imgproc.arcLength(c2f, true);
                    MatOfPoint2f approx = new MatOfPoint2f();
                    Imgproc.approxPolyDP(c2f, approx, epsilon, true);

                    List<Point> points = new ArrayList<>();
                    for (int j = 0; j < approx.rows(); j++){
                        points.add(approx.toList().get(j));
                    }

                    if(points.size()>4 && points.size()<=6){
                        Map<Double,Double> vertices = new TreeMap<>();
                        for(Point point:points){
                            vertices.put(point.y, point.x);
                        }
                        List<Double> ysSorted = (List<Double>) vertices.keySet();

                        List<Point> pointsSorted = new ArrayList<>();
                        for(Double val:ysSorted){
                            pointsSorted.add(new Point(vertices.get(val), val));
                        }

                        Point firstHighestPoint = pointsSorted.get(0);
                        Point secondHighestPoint = pointsSorted.get(1);
                        Point thirdHighestPoint = pointsSorted.get(2);

                        Imgproc.drawMarker(input, firstHighestPoint,new Scalar(255,0,0));
                        Imgproc.drawMarker(input, secondHighestPoint,new Scalar(255,0,0));
                        Imgproc.drawMarker(input, thirdHighestPoint,new Scalar(255,0,0));


                        List<Point> longestLine = new ArrayList<>();
                        double longestDistance = 0.0;
                        for(int i =1; i < points.size(); i++){
                            if(i==1){
                                longestLine.add(points.get(0));
                                longestLine.add(points.get(1));
                                longestDistance = Math.hypot(points.get(i).x - points.get(0).x,points.get(i).y - points.get(0).y);
                            }else{
                                if(longestDistance<Math.hypot(points.get(i).x - points.get(i-1).x,points.get(i).y - points.get(i-1).y)){
                                    longestDistance = Math.hypot(points.get(i).x - points.get(i-1).x,points.get(i).y - points.get(i-1).y);
                                    longestLine.clear();
                                    longestLine.add(points.get(i-1));
                                    longestLine.add(points.get(i));
                                }

                            }
                        }


                        Point centerOfSample = new Point((secondHighestPoint.x+thirdHighestPoint.x)/2,
                                (secondHighestPoint.y+thirdHighestPoint.y)/2);
                        Imgproc.drawMarker(input, centerOfSample, new Scalar(255,255,0));
                        List<Object> singleSampleData = new ArrayList<>();
                        Point sampleCentroid = onScreen2RealWorld(centerOfSample);
                        singleSampleData.add(sampleCentroid);
                        if(longestLine.get(1).y<longestLine.get(0).y){
                                Collections.reverse(longestLine);
                        }
                        double sampleAngleOfRotation = angle3pt(longestLine.get(1),longestLine.get(0),
                                new Point(width, longestLine.get(0).y));
                        singleSampleData.add(sampleAngleOfRotation);
                        samplesData.add(singleSampleData);
                }
            }
        }
        List<Object> closest = new ArrayList<>();
        double shortestDistance = 0;
        for(int i =0; i<samplesData.size();i++){
            Point point = (Point) samplesData.get(0);
            if(i==0){
                closest = samplesData.get(i);

                shortestDistance =  Math.hypot(position.get(0) - point.x, position.get(1)-point.y);
            }else{
                if(shortestDistance>Math.hypot(position.get(0) - point.x, position.get(1)-point.y)){
                    closest=samplesData.get(i);
                    shortestDistance = Math.hypot(position.get(0) - point.x, position.get(1)-point.y);
                }
            }
        }
        if(closest.size==2){
            centroid = (Point) closest.get(0);
            angleOfRotation = (double) closest.get(0);
        }
        long endTime = System.nanoTime();
        timeTakenMili = endTime-startTime;
        return input;
        }
        private Point onScreen2RealWorld(Point centroid){
            boolean xIsNegative = false;
            double yAngleDown = ( (centroid.y - this.screenCenterY) /this.screenCenterY)*60;
            double xAngle = ( (centroid.x - this.screenCenterX) /this.screenCenterX)*60;
            if( xAngle <0){
                xIsNegative = true;
                xAngle = -xAngle;
            }
            double yRealWorld =  distanceOffGround/Math.tan(Math.toRadians(yAngleDown));
            double xRealWorld = Math.tan(xAngle)*yRealWorld;
            if(xIsNegative)
                xRealWorld = -xRealWorld;
            return transformPosition(new  Point(position.get(0), position.get(1)), position.get(2),new Point( xRealWorld,yRealWorld));



        }
        private Point transformPosition(Point robot, double degreesR, Point offset) {
            /*
             * Transform the object's position relative to the robot to the field's coordinate system.
             *
             * Parameters:
             * xR, yR: Robot's position in the field's coordinate system
             * degreesR: Robot's heading (in degrees) relative to the field
             * xO, yO: Object's position relative to the robot (in robot's local coordinate system)
             *
             * Returns:
             * (xField, yField): Object's position in the field's coordinate system
             */

            // Convert degrees to radians
            double thetaR = Math.toRadians(degreesR);

            // Create the rotation matrix based on robot's heading
            double[][] rotationMatrix = new double[][]{
                    {Math.cos(thetaR), -Math.sin(thetaR)},
                    {Math.sin(thetaR), Math.cos(thetaR)}
            };

            // Rotate the object's local coordinates relative to the robot
            double objectRotatedX = rotationMatrix[0][0] * offset.x + rotationMatrix[0][1] * offset.y;
            double objectRotatedY = rotationMatrix[1][0] * offset.x + rotationMatrix[1][1] * offset.y;

            // Translate by the robot's position in the field's coordinate system
            double xField = robot.x + objectRotatedX;
            double yField = robot.y + objectRotatedY;
            ArrayList<Double> coordinates = new ArrayList<>();
            coordinates.add(xField);
            coordinates.add(yField);
            // Return the result as an array
            return new Point(xField,yField);
        }
        private double angle3pt(Point a,Point b, Point c){
            double angle = Math.toDegrees(
                    Math.atan2(c.y-b.y, c.x- b.x) -Math.atan2(a.y-b.y,
                            a.x-b.x));
            if( angle<0){
                return 360+angle;
            }
            if(angle>180)
                angle-=180;
            return angle;
        }
        private Mat preprocess(Mat frame){
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame,Imgproc.COLOR_BGR2HSV);
            // Scalars used to detect the yellow samples
            Scalar lowerYellow = new Scalar(5, 139, 109);
            Scalar upperYellow = new Scalar(31, 255, 255);


            Core.inRange(hsvFrame,lowerYellow,upperYellow,frame);

            //Scalars used to detect the lower red of the samples



            Point anchorPoint = new Point(0, 0);
            Imgproc.erode(frame,frame, Imgproc.getStructuringElement(
                    Imgproc.MORPH_RECT, new Size(5, 5)), anchorPoint,1);
            Imgproc.dilate(frame,frame,Imgproc.getStructuringElement(
                    Imgproc.MORPH_RECT, new Size(5, 5)),anchorPoint,1);
            Imgproc.erode(frame,frame, Imgproc.getStructuringElement(
                    Imgproc.MORPH_RECT, new Size(5, 5)), anchorPoint,1);
            Imgproc.dilate(frame,frame,Imgproc.getStructuringElement(
                    Imgproc.MORPH_RECT, new Size(5, 5)),anchorPoint,2);

            return frame;



        }

    }

}
