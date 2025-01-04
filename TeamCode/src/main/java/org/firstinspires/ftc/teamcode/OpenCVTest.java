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
import java.util.List;
import java.util.Map;
import java.util.TreeMap;


@TeleOp(name="OpenCV testing")
public class OpenCVTest extends LinearOpMode {
    final int width = 1920;
    final int height = 1080;
    private OpenCvCamera controlHubCam;
    private final double DistanceOffGround = 10;


    SampleDetection cvpipline;

    List<Double> closest ;
    double distance;
    private List<Double> getPosition(){
        return Arrays.asList(0.0,0.0,0.0);
    }

    @Override
    public void runOpMode(){
        HardwareMap hwmap = hardwareMap;
        initOpenCV();
        waitForStart();
        Gamepad gamepad1 = new Gamepad();


        while (opModeIsActive()){
            cvpipline.pos(getPosition());
            if( gamepad1.x){
                List<ArrayList<Double>> samples = cvpipline.samples;

                for(int i=0;i<samples.size(); i++){
                    List<Double> sample = samples.get(i);
                    if (i==0){
                         closest = sample;

                         distance = Math.hypot(sample.get(0)-getPosition().get(0),sample.get(1)-getPosition().get(1));
                    }else{

                        if(distance<Math.hypot(sample.get(0)-getPosition().get(0),sample.get(1)-getPosition().get(1))){
                            distance = Math.hypot(sample.get(0)-getPosition().get(0),sample.get(1)-getPosition().get(1));
                            closest = sample;
                        }
                    }
                }
                telemetry.addData("Closest Sample [x,y,angle]: ", closest.toString());
            }

        }
    }


    private void initOpenCV() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        cvpipline = new SampleDetection();
        controlHubCam.setPipeline(cvpipline);


        controlHubCam.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);


}
class SampleDetection extends OpenCvPipeline{
    final double width = 1920;
    final double height = 1080;
    final double screenCenterX = width/2;
    final double screenCenterY = height/2;
     final double distanceOffGround = 10.5;
    List<Double> pos = Arrays.asList(0.0,0.0,0.0);
    List<ArrayList<Double>>  samples = new ArrayList<>();

    @Override
    public Mat processFrame(Mat input){
        List<ArrayList<Double>>  samples = new ArrayList<>();
        List<MatOfPoint> contours = getContours(input);
        for(MatOfPoint c:contours){
            if( Imgproc.contourArea(c) >=15000){
                MatOfPoint2f c2f = new MatOfPoint2f(c.toArray());
                double epsilon = 0.0129032258 * Imgproc.arcLength(c2f, true);
                MatOfPoint2f approx = new MatOfPoint2f();
                Imgproc.approxPolyDP(c2f, approx, epsilon, true);

                List<Point> points = new ArrayList<>();
                for (int j = 0; j < approx.rows(); j++) {
                    points.add(approx.toList().get(j));
                }
                if(points.size()>4&points.size()<=6){
                    Map<Double,Double>  vertices = new TreeMap<>();
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

                    double centerX = (secondHighestPoint.x+thirdHighestPoint.x)/2;
                    double centerY = (secondHighestPoint.y+thirdHighestPoint.y)/2;

                    ArrayList<Double> fieldCentricCoordinates = onScreen2RealWorld(centerX,centerY);
                    List<Double> side = new ArrayList<>();
                    side.add(this.width);
                    side.add(firstHighestPoint.y);


                    List<Integer> secondHighestPointli = new ArrayList<Integer>();
                    secondHighestPointli.add((int) secondHighestPoint.x);
                    secondHighestPointli.add( (int) secondHighestPoint.y);


                    List<Integer> FirstHighestPointli = new ArrayList<Integer>();
                    FirstHighestPointli.add((int) firstHighestPoint.x);
                    FirstHighestPointli.add((int) firstHighestPoint.y);

                    fieldCentricCoordinates.add(angle3pt(secondHighestPointli,FirstHighestPointli,  side));
                    samples.add(fieldCentricCoordinates);
                }


            }
        }
        return input;
    }
    public void pos(List<Double> pos){
        this.pos = pos;
    }

    public List<ArrayList<Double>> getContours(){
        return samples;
    }
    private ArrayList<Double> onScreen2RealWorld(double x, double y){
        boolean xIsNegative = false;
        double yAngleDown = ((double) (y - this.screenCenterY) /this.screenCenterY)*60;
        double xAngleDown = ((double) (x - this.screenCenterX) /this.screenCenterX)*60;
        if( xAngleDown<0){
            xIsNegative = true;
            xAngleDown = -xAngleDown;
        }
        double yRealWorld =  this.distanceOffGround/Math.tan(Math.toRadians(yAngleDown));
        double xRealWorld = Math.tan(xAngleDown)*yRealWorld;

        return transformPosition(pos.get(0), pos.get(1), pos.get(2), xRealWorld,yRealWorld);



    }

    private   ArrayList<Double> transformPosition(double xR, double yR, double degreesR, double xO, double yO) {
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
        double objectRotatedX = rotationMatrix[0][0] * xO + rotationMatrix[0][1] * yO;
        double objectRotatedY = rotationMatrix[1][0] * xO + rotationMatrix[1][1] * yO;

        // Translate by the robot's position in the field's coordinate system
        double xField = xR + objectRotatedX;
        double yField = yR + objectRotatedY;
        ArrayList<Double> coordinates = new ArrayList<>();
        coordinates.add(xField);
        coordinates.add(yField);
        // Return the result as an array
        return coordinates;
    }


    private double angle3pt(List<Integer> a,List<Integer> b, List<Double> c){
        double angle = Math.toDegrees(
                Math.atan2(c.get(1)-b.get(1), c.get(0)- b.get(0)) -Math.atan2(a.get(1)-b.get(1),
                        a.get(0)-b.get(0)));
        if( angle<0){
            return 360+angle;
        }

        return angle;
    }
    private ArrayList<MatOfPoint> getContours(Mat frame){
        Mat hsvframe = new Mat();
        Imgproc.cvtColor(frame, hsvframe,Imgproc.COLOR_BGR2HSV);
        // Scalars used to detect the yellow samples
        Scalar lowerYellow = new Scalar(5, 139, 109);
        Scalar upperYellow = new Scalar(31, 255, 255);

        Mat yellowMask = new Mat();
        Core.inRange(hsvframe,lowerYellow,upperYellow,yellowMask);

        //Scalars used to detect the lower red of the samples
        Scalar lowerRed1 = new Scalar(0, 100, 100);
        Scalar upperRed1 = new Scalar(10, 255, 255);

        Mat LowerRedMask = new Mat();
        Core.inRange(hsvframe,lowerRed1,upperRed1, LowerRedMask);

        //Scalars used for the upper red of the samples
        Scalar lowerRed2 = new Scalar(170, 100, 100);
        Scalar upperRed2 = new Scalar(180, 255, 255);

        Mat UpperRedMask = new Mat();
        Core.inRange(hsvframe,lowerRed2,upperRed2,UpperRedMask);

        Mat RedMask = new Mat();
        Core.bitwise_or(UpperRedMask,LowerRedMask, RedMask);

        Mat Mask = new Mat();
        Core.bitwise_or(RedMask,yellowMask, Mask);


        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Point anchorPoint = new Point(0, 0);
        Imgproc.erode(Mask,Mask, kernel, anchorPoint,1);
        Imgproc.dilate(Mask,Mask,kernel,anchorPoint,1);
        Imgproc.erode(Mask,Mask, kernel, anchorPoint,1);
        Imgproc.dilate(Mask,Mask,kernel,anchorPoint,2);

        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(Mask, contours,new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        return contours;



    }

    }
}

