package org.firstinspires.ftc.teamcode.modules;

import android.graphics.Canvas;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
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


public class VisionPipeline implements VisionProcessor {
    final double width = 1920;
    final double height = 1080;
    final double screenCenterX = width / 2;
    final double screenCenterY = height / 2;
    final double distanceOffGround = 8.9;
    Mat hsvFrame = new Mat();
    Mat mask = new Mat();
    Mat redUpper = new Mat();
    Mat redLower = new Mat();
    private final double lowerX =  0* width;
    private final double upperX = 1 * width;
    private final double lowerY = .5 * height;
    private final double upperY = 1.0 * height;
    public Point centroid = new Point();
    public double angleOfRotation = 0;
    public double timeTakenMili = 0;
    public List<Double> position = Arrays.asList(0.0, 0.0, 0.0);
    private final Mat cameraMatrix = new Mat();
    private final Mat distCoeffs = new Mat();
    private Mat editingFrame = new Mat();

    //0=red 1=blue 2=yellow
    private final int color;

    public boolean nothingThere;


    // Scalars used to detect the yellow samples
    Scalar lowerYellow = new Scalar(5, 139, 109);

    Scalar upperYellow = new Scalar(31, 255, 255);

    //scalars used to detect the red samples
    Scalar upperRedHigh = new Scalar(180,255,255);
    Scalar upperRedLow = new Scalar(152,123,31);
    Scalar lowerRedHigh = new Scalar (13,255,255);
    Scalar lowerRedLow = new Scalar(0,116,65);
    //scalars used to detect the blue samples
    Scalar lowerBlue = new Scalar(90,75,168);
    Scalar upperBlue = new Scalar(118,255,255);

    //
    public double distanceForward = 0;


    public VisionPipeline(int color) {
        this.color = color;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        cameraMatrix.put(0, 0, 595.37521152, 0.0, 952.22722088,
                0.0, 597.10091695, 488.29707956,
                0.0, 0.0, 1.0
        );
        distCoeffs.put(0, 0, -0.00593377, -0.00816853, 0.00092361, -0.00103652, -0.00245282);
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        editingFrame = frame;
        //Calib3d.undistort(frame,editingFrame,cameraMatrix,distCoeffs);
        long startTime = System.nanoTime();
        List<List<Object>> samplesData = new ArrayList<>();
        preprocess(editingFrame);
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);


        for (MatOfPoint c : contours) {
            if (Imgproc.contourArea(c) > 1000) {
                MatOfPoint2f c2f = new MatOfPoint2f(c.toArray());
                double epsilon = 0.0129032258 * Imgproc.arcLength(c2f, true);
                MatOfPoint2f approx = new MatOfPoint2f();
                Imgproc.approxPolyDP(c2f, approx, epsilon, true);

                List<Point> points = new ArrayList<>();
                for (int j = 0; j < approx.rows(); j++) {
                    points.add(approx.toList().get(j));
                }


                if (points.size() == 6) {


                    List<Point> longestLine = new ArrayList<>();
                    double longestDistance = 0.0;
                    for (int i = 1; i < points.size(); i++) {
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
                    for (int i = 1; i < points.size(); i++) {
                        if (!initUsed && !longestLine.contains(points.get(i))) {
                            secondLongestLine.add(points.get(i));
                            secondLongestLine.add(points.get(i - 1));
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
                    if ((longestLine.get(0).y - longestLine.get(1).y) / 2 > (secondLongestLine.get(0).y - secondLongestLine.get(1).y) / 2) {
                        lineUsed = secondLongestLine;
                    }

                    if (lineUsed.get(1).y < lineUsed.get(0).y) {
                        Collections.reverse(lineUsed);
                    }
                    Point pointUsed = new Point();
                    longestDistance = 0;
                    for (int i = 0; i < points.size(); i++) {
                        if (i == 0) {
                            pointUsed = points.get(i);
                            longestDistance = Math.hypot(points.get(i).x - lineUsed.get(0).x, points.get(i).y - lineUsed.get(0).y);
                        } else {
                            if (longestDistance < Math.hypot(points.get(i).x - lineUsed.get(0).x, points.get(i).y - lineUsed.get(0).y)) {
                                pointUsed = points.get(i);
                                longestDistance = Math.hypot(points.get(i).x - lineUsed.get(0).x, points.get(i).y - lineUsed.get(0).y);

                            }
                        }

                    }

                    Point screencenter = new Point((lineUsed.get(1).x + pointUsed.x) / 2, (lineUsed.get(1).y + pointUsed.y) / 2);
                    if (screencenter.x < lowerX || screencenter.x > upperX || screencenter.y < lowerY || screencenter.y > upperY) {
                        continue;
                    }
                    Imgproc.drawContours(editingFrame, Collections.singletonList(c), -1, new Scalar(0, 0, 255));
                    Point rWPos = onScreen2RealWorld(screencenter);
                    List<Object> sampleData = new ArrayList<>();
                    sampleData.add(rWPos);
                    double angle = angle3pt(lineUsed.get(1), lineUsed.get(0), new Point(width, lineUsed.get(0).y));
                    sampleData.add(angle);
                    samplesData.add(sampleData);
                }
            }
        }


        List<Object> closest = new ArrayList<>();
        double shortestDistance = 0;
        for (int i = 0; i < samplesData.size(); i++) {
                if((((Point) samplesData.get(i).get(0)).x)<7.5&&((Point) samplesData.get(i).get(0)).x<5.5 && ((Point) samplesData.get(i).get(0)).y<20){
                    distanceForward = ((Point) samplesData.get(i).get(0)).y-5.03937;
                    angleOfRotation = (double) samplesData.get(i).get(1);

                }
        }
        nothingThere = false;//samplesData.isEmpty();
        long endTime = System.nanoTime();
        timeTakenMili = (double) (endTime - startTime) / 1000000;
        return editingFrame;
    }
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        canvas.drawCircle(500, 500, 4, new Paint(3));
    }

    private void preprocess(Mat frame) {

        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_RGB2HSV);



        if(color==0) {
            //reds
            //lower Red
            Core.inRange(hsvFrame,lowerRedLow,lowerRedHigh,redLower);
            //higher Red
            Core.inRange(hsvFrame, upperRedLow,upperRedHigh,redUpper);
            //combining both masks
            Core.bitwise_or(redUpper,redLower,mask);

        }else if(color==1){
            //yellow
            Core.inRange(hsvFrame,lowerBlue,upperBlue,mask);
        } else if (color==2) {
            //Yellow
            Core.inRange(hsvFrame, lowerYellow, upperYellow, mask);
        }
        //Scalars used to detect the lower red of the samples

        Point anchorPoint = new Point(-1, -1);
        Imgproc.erode(mask, mask, Imgproc.getStructuringElement(
                Imgproc.MORPH_RECT, new Size(10, 10)), anchorPoint, 1);
        Imgproc.dilate(mask, mask, Imgproc.getStructuringElement(
                Imgproc.MORPH_RECT, new Size(10, 10)), anchorPoint, 1);


    }

    private Point onScreen2RealWorld(Point centroid) {
        boolean xIsNegative = false;
        double yAngleDown = 90 - ((centroid.y - this.screenCenterY) / this.screenCenterY) * 60;
        double xAngle = 90 - ((centroid.x - this.screenCenterX) / this.screenCenterX) * 60;
        if (xAngle < 0) {
            xIsNegative = true;
            xAngle = -xAngle;
        }
        double yRealWorld = distanceOffGround / Math.tan(Math.toRadians(yAngleDown));
        double xRealWorld = Math.tan(xAngle) * yRealWorld;
        if (xIsNegative)
            xRealWorld = -xRealWorld;
        return  new Point(xRealWorld, yRealWorld);


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
        return new Point(xField, yField);
    }

    private double angle3pt(Point a, Point b, Point c) {
        double angle = Math.toDegrees(
                Math.atan2(c.y - b.y, c.x - b.x) - Math.atan2(a.y - b.y,
                        a.x - b.x));
        if (angle < 0) {
            return 360 + angle;
        }
        if (angle > 180)
            angle -= 180;
        return angle;
    }

}
