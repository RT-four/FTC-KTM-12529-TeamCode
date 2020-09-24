package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;


/**
 * Created by maryjaneb  on 11/13/2016.
 *
 * nerverest ticks
 * 60 1680
 * 40 1120
 * 20 560
 *
 * monitor: 640 x 480
 */
@Autonomous(name= "AutoMoveFoundation", group="Sky autonomous")
//@Disabled
public class AutoMoveFoundation extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor backLeft     = null; //rear left
    private DcMotor backRight    = null; //rear right
    private DcMotor frontLeft    = null; //front left
    private DcMotor frontRight   = null; //front right

    private Servo servoRight        = null;
    private Servo servoLeft         = null;

    private final int encoderTicks = 1120;
    private final double wheelDiameter = 3.85827;//in inches

    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    public static int valMid = -1;
    public static int valLeft = -1;
    public static int valRight = -1;

    public static float rectHeight = .6f/8f;
    public static float rectWidth = 1.5f/8f;

    public static float offsetX = 1.65f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    public static float offsetY = 1.5f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    public static float[] midPos = {4f/8f+offsetX, (4f+offsetY)/8f};//0 = col, 1 = row
    public static float[] leftPos = {2f/8f+offsetX, (4f+offsetY)/8f};
    public static float[] rightPos = {6f/8f+offsetX, (4f+offsetY)/8f};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    public final int rows = 640;
    public final int cols = 480;

    OpenCvCamera phoneCam;

//    x, y = distance in x,y direction, angle = angle for rotation, power = motor power/speed
//    for now, use only one direction at a time
    public void move(double x, double y, double angle, double power) {
       backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

       backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

       //y
       double distancePerRotationY = 3.1415 * wheelDiameter; //pi * diameter (inches) = circumference
       double rotationsY = y/distancePerRotationY; //distance / circumference (inches)
       int encoderTargetY = (int)(rotationsY*encoderTicks);

       //x
       double distancePerRotationX = 13.5; //distance per rotations is different than circumference when strafing (inches)
       double rotationsX = x/distancePerRotationX; //distance / circumference (inches)
       int encoderTargetX = (int)(rotationsX*encoderTicks);

       //angle
       double ticksPerRotation = 0;//measure how many ticks for a 360 rotation
       double rotationsA = angle/360;
       int encoderTargetA = (int)(rotationsA*ticksPerRotation);

       if(opModeIsActive()) {
         backLeft.setTargetPosition(encoderTargetY-encoderTargetX+encoderTargetA);
         backRight.setTargetPosition(encoderTargetY+encoderTargetX-encoderTargetA);
         frontLeft.setTargetPosition(encoderTargetY+encoderTargetX+encoderTargetA);
         frontRight.setTargetPosition(encoderTargetY-encoderTargetX-encoderTargetA);

         backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

         backLeft.setPower(Math.abs(power));//childproof. must have always positive power
         backRight.setPower(Math.abs(power));
         frontLeft.setPower(Math.abs(power));
         frontRight.setPower(Math.abs(power));

         while(backLeft.isBusy() || backRight.isBusy() || frontLeft.isBusy() || frontRight.isBusy()) {
           //wait till motor finishes working
           telemetry.addData("Path", "Driving");
           telemetry.update();
         }

         backLeft.setPower(0);
         backRight.setPower(0);
         frontLeft.setPower(0);
         frontRight.setPower(0);

         telemetry.addData("Path", "Complete");
         telemetry.update();

         backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       }
   }


    @Override
    public void runOpMode() throws InterruptedException {
       backLeft        = hardwareMap.dcMotor.get("left_drive");
       backRight       = hardwareMap.dcMotor.get("right_drive");
       frontLeft       = hardwareMap.dcMotor.get("front_left");
       frontRight      = hardwareMap.dcMotor.get("front_right");
       servoLeft       = hardwareMap.servo.get("servoLeft");
       servoRight      = hardwareMap.servo.get("servoRight");

       backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
       frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
       backRight.setDirection(DcMotorSimple.Direction.FORWARD);
       frontRight.setDirection(DcMotorSimple.Direction.FORWARD);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        //phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.

        waitForStart();
        runtime.reset();
        while (opModeIsActive())
        {
            telemetry.addData("Values", valLeft+"   "+valMid+"   "+valRight);
            telemetry.addData("Height", rows);
            telemetry.addData("Width", cols);

            telemetry.update();
            sleep(100);
            //call movement functions
//            strafe(0.4, 200);
//            moveDistance(0.4, 700);

        }
    }





//Detector code after this
//..............................................................................

    //detection pipeline
    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage
        {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private Stage stageToRenderToViewport = Stage.detection;
        private Stage[] stages = Stage.values();

        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input)
        {
            contoursList.clear();
            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixMid = thresholdMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
            valMid = (int)pixMid[0];

            double[] pixLeft = thresholdMat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
            valLeft = (int)pixLeft[0];

            double[] pixRight = thresholdMat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
            valRight = (int)pixRight[0];

            //create three points
            Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
            Point pointLeft = new Point((int)(input.cols()* leftPos[0]+input.cols()), (int)(input.rows()* leftPos[1]));
            Point pointRight = new Point((int)(input.cols()* rightPos[0]-input.cols()), (int)(input.rows()* rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointLeft,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle


//            MatOfPoint2f approxCurve = new MatOfPoint2f();
//            //For each contour found
//            for (int i=0; i<contoursList.size(); i++)
//            {
//                boolean skyFound = false;
//                //Convert contours(i) from MatOfPoint to MatOfPoint2f
//                MatOfPoint2f contour2f = new MatOfPoint2f( contoursList.get(i).toArray() );
//                //Processing on mMOP2f1 which is in type MatOfPoint2f
//                double approxDistance = Imgproc.arcLength(contour2f, true)*0.02;
//                Imgproc.approxPolyDP(contour2f, approxCurve, approxDistance, true);
//
//                //Convert back to MatOfPoint
//                MatOfPoint points = new MatOfPoint(approxCurve.toArray() );
//                // Get bounding rect of contour
//                Rect rect = Imgproc.boundingRect(points);
//
//                //if the contour is in specified location AND one of the three points' value = 0, 0 means skystone
//                //this is extra security in case detected color is not a skystone
//                if(valMid == 0 || valLeft == 0 || valRight == 0)
//                    skyFound = true;
//                else
//                    skyFound = false;
//
//                if((rect.contains(pointMid) || rect.contains(pointLeft) || rect.contains(pointRight))) {
//                    Imgproc.rectangle(all,
//                            new Point(rect.x, rect.y),
//                            new Point(rect.x + rect.width, rect.y + rect.height),
//                            new Scalar(255, 0, 0, 255), 3);
//                    break;
//                }
//
//            }

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols()*(leftPos[0]-rectWidth/2)+input.cols()*offsetX,
                            input.rows()*(leftPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(leftPos[0]+rectWidth/2)+input.cols()*offsetX,
                            input.rows()*(leftPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols()*(midPos[0]-rectWidth/2)+input.cols()*offsetX,
                            input.rows()*(midPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(midPos[0]+rectWidth/2)+input.cols()*offsetX,
                            input.rows()*(midPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols()*(rightPos[0]-rectWidth/2)+input.cols()*offsetX,
                            input.rows()*(rightPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(rightPos[0]+rectWidth/2)+input.cols()*offsetX,
                            input.rows()*(rightPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);

            switch (stageToRenderToViewport)
            {
                case THRESHOLD:
                {
                    return thresholdMat;
                }

                case detection:
                {
                    return all;
                }

                case RAW_IMAGE:
                {
                    return input;
                }

                default:
                {
                    return input;
                }
            }
        }

    }
}
