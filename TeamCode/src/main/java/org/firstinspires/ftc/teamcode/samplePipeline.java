
package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import android.graphics.Paint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;


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
        //Gamepad gamepad1 = new Gamepad();
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
        yellowVisionPortal.setROI(.25,.75,.5,1.0);
        visionPortal = new VisionPortal.Builder().
                addProcessor(yellowVisionPortal)
                .setCameraResolution(new android.util.Size(1920, 1080))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam1"))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();


    }





}
class YellowVisionPortal implements VisionProcessor{
    final double width = 1920;
    final double height = 1080;
    final double screenCenterX = width/2;
    final double screenCenterY = height/2;
    final double distanceOffGround = 8.9;
    Mat hsvFrame = new Mat();
    Mat mask = new Mat();
    private double lowerX;
    private double upperX;
    private double lowerY;
    private double upperY;
    public Point centroid = new Point();
    public double angleOfRotation = 0;
    public double timeTakenMili = 0;
    public List<Double> position = Arrays.asList(0.0, 0.0, 0.0);


    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        long startTime = System.nanoTime();
        List<List<Object>> samplesData = new ArrayList<>();
        preprocess(frame);

        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);



        for(MatOfPoint c: contours){
            if(Imgproc.contourArea(c)>1000){
                MatOfPoint2f c2f = new MatOfPoint2f(c.toArray());
                double epsilon = 0.0129032258 * Imgproc.arcLength(c2f, true);
                MatOfPoint2f approx = new MatOfPoint2f();
                Imgproc.approxPolyDP(c2f, approx, epsilon, true);

                List<Point> points = new ArrayList<>();
                for (int j = 0; j < approx.rows(); j++){
                    points.add(approx.toList().get(j));
                }


                if(points.size()>4 && points.size()<=6){


                    List<Point> longestLine = new ArrayList<>();
                    double longestDistance = 0.0;
                    for(int i =1; i < points.size(); i++) {
                        if (i == 1) {
                            longestLine.add(points.get(0));
                            longestLine.add(points.get(1));
                            longestDistance = Math.hypot(points.get(i).x - points.get(0).x, points.get(i).y - points.get(0).y);
                        } else {
                            if (longestDistance < Math.hypot(points.get(i).x - points.get(i - 1).x, points.get(i).y - points.get(i - 1).y)) {
                                longestDistance = Math.hypot(points.get(i).x - points.get(i - 1).x, points.get(i).y - points.get(i - 1).y);
                                longestLine.clear();
                                longestLine.add(points.get(i - 1));
                                longestLine.add(points.get(i));
                            }
                        }
                    }


                    List<Point> secondLongestLine = new ArrayList<>();
                    double secondLongestDistance = 0;
                    boolean initUsed = false;
                    for(int i =1; i < points.size(); i++) {
                        if (!initUsed  && !longestLine.contains(points.get(i))) {
                            secondLongestLine.add(points.get(i));
                            secondLongestLine.add(points.get(i-1));
                            secondLongestDistance = Math.hypot(points.get(i).x - points.get(0).x, points.get(i).y - points.get(0).y);
                            initUsed = true;
                        } else {
                            if (secondLongestDistance < Math.hypot(points.get(i).x - points.get(i - 1).x, points.get(i).y - points.get(i - 1).y) && !longestLine.contains(points.get(i))) {
                                secondLongestDistance = Math.hypot(points.get(i).x - points.get(i - 1).x, points.get(i).y - points.get(i - 1).y);
                                secondLongestLine.clear();
                                secondLongestLine.add(points.get(i - 1));
                                secondLongestLine.add(points.get(i));
                            }
                        }
                    }

                    List<Point> lineUsed = longestLine;
                    if((longestLine.get(0).y-longestLine.get(1).y)/2>(secondLongestLine.get(0).y-secondLongestLine.get(1).y)/2){
                        lineUsed = secondLongestLine;
                    }

                    if(lineUsed.get(1).y<lineUsed.get(0).y){
                        Collections.reverse(lineUsed);
                    }
                    Point pointUsed = new Point();
                    longestDistance = 0;
                    for(int i=0;i<points.size(); i++){
                        if(i==0){
                            pointUsed = points.get(i);
                            longestDistance = Math.hypot(points.get(i).x-lineUsed.get(0).x,points.get(i).y-lineUsed.get(0).y);
                        }else{
                            if(longestDistance< Math.hypot(points.get(i).x-lineUsed.get(0).x,points.get(i).y-lineUsed.get(0).y)){
                                pointUsed = points.get(i);
                                longestDistance  =  Math.hypot(points.get(i).x-lineUsed.get(0).x,points.get(i).y-lineUsed.get(0).y);

                            }
                        }

                    }

                    Point screencenter = new Point((lineUsed.get(1).x+pointUsed.x)/2, (lineUsed.get(1).y+pointUsed.y)/2);
                    if(screencenter.x<lowerX || screencenter.x>upperX || screencenter.y<lowerY || screencenter.y>upperY){
                        continue;
                    }
                    Imgproc.drawContours(frame, Collections.singletonList(c), -1, new Scalar(0,0,255));
                    Point rWPos = onScreen2RealWorld(screencenter);
                    List<Object> sampleData= new ArrayList<>();
                    sampleData.add(rWPos);
                    double angle = angle3pt(lineUsed.get(1), lineUsed.get(0), new Point(width,lineUsed.get(0).y));
                    sampleData.add(angle);
                    samplesData.add(sampleData);
                }
            }
        }



        List<Object> closest = new ArrayList<>();
        double shortestDistance = 0;
        for(int i =0; i<samplesData.size();i++){
            Point point = (Point) samplesData.get(i).get(0);
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

        if(closest.size()==2){
            centroid = (Point) closest.get(0);
            angleOfRotation = (double) closest.get(1);
        }
        Imgproc.drawMarker(frame,centroid, new Scalar(255, 192, 203));
        long endTime = System.nanoTime();
        timeTakenMili = (double) (endTime - startTime) /1000000;
        return mask;
    }
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        canvas.drawCircle(500,500, 4, new Paint(3));
    }
    private void preprocess(Mat frame){
        Imgproc.cvtColor(frame, hsvFrame,Imgproc.COLOR_RGB2HSV);

        // Scalars used to detect the yellow samples
        Scalar lowerYellow = new Scalar(5, 139, 109);

        Scalar upperYellow = new Scalar(31, 255, 255);


        Core.inRange(hsvFrame,lowerYellow,upperYellow,mask);

        //Scalars used to detect the lower red of the samples



        Point anchorPoint = new Point(-1, -1);
        Imgproc.erode(mask,mask, Imgproc.getStructuringElement(
                Imgproc.MORPH_RECT, new Size(10,10)), anchorPoint,1);
        Imgproc.dilate(mask,mask,Imgproc.getStructuringElement(
                Imgproc.MORPH_RECT, new Size(10,10)),anchorPoint,1);




    }
    private Point onScreen2RealWorld(Point centroid){
        boolean xIsNegative = false;
        double yAngleDown = 90-( (centroid.y - this.screenCenterY) /this.screenCenterY)*60;
        double xAngle = 90-( (centroid.x - this.screenCenterX) /this.screenCenterX)*60;
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
    public void setROI(double lowerX,double upperX,double lowerY,double upperY){
        this.lowerX = lowerX*width;
        this.upperX = upperX*width;
        this.lowerY = lowerY*height;
        this.upperY = upperY*height;
    }
}
