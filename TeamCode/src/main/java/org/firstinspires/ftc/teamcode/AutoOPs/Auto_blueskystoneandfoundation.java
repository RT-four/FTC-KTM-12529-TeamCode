package org.firstinspires.ftc.teamcode.AutoOPs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robot;

/**
 * Created by maryjaneb  on 11/13/2016.
 *
 * nerverest ticks
 * 60 1680
 * 40 1120
 * 20 560
 *
 * monitor: 640 x 480
 *YES
 */
@Autonomous(name= "Auto_blueskystoneandfoundation", group="AutoOP")
//@Disabled//comment out this line before using
public class Auto_blueskystoneandfoundation extends robot {
    private ElapsedTime runtime = new ElapsedTime();

    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;

    private static float rectHeight = .5f/8f;//,8 стандарт
    private static float rectWidth = 1.4f/8f;

    private static float offsetX = 0f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {4f/8f+offsetX, 4.6f/8f+offsetY};//0 = col, 1 = row
    private static float[] leftPos = {2.35f/8f+offsetX, 4.6f/8f+offsetY};//изменение параметров= передвижение всех объектов
    private static float[] rightPos = {5.65f/8f+offsetX, 4.6f/8f+offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;

    OpenCvCamera phoneCam;

    @Override
    public void runOpMode() {
        initHW(hardwareMap);
        //  wow.init(hardwareMap);
        s5Shovel.setPosition(0);
        s4Kicker.setPosition(0);
        s3Rotation.setPosition(0);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.
        //while (opModeIsActive()) {
        telemetry.addData("Values", valLeft+"   "+valMid+"   "+valRight);
        telemetry.addData("Height", rows);
        telemetry.addData("Width", cols);

        telemetry.update();
        sleep(100);
        //call movement functions
//            strafe(0.4, 200);
//            moveDistance(0.4, 700);

        //}
        BNO055IMU imu;
        Orientation angles;
        String pologenieskystone="";
        s3Rotation.setPosition(0.7);
        BNO055IMU.Parameters parametrs= new BNO055IMU.Parameters();
        parametrs.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu rev");
        waitForStart();{
            imu.initialize(parametrs);
            angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            int l=0,m=0,r=0,pologenieskystone1=0;
            if(valLeft==0&&valMid==255&&valRight==255){
                pologenieskystone="left";
                l=0;
                m=255;
                r=255;
                pologenieskystone1=1;
            }
            if(valLeft==255&&valMid==0&&valRight==255){
                pologenieskystone="middle";
                l=255;
                m=0;
                r=255;
                pologenieskystone1=1;
            }
            if(valLeft==255&&valMid==255&&valRight==0){
                pologenieskystone="right";
                l=255;
                m=255;
                r=0;
                pologenieskystone1=1;
            }
            double voltage=BatteryVoltage();
            double koeff=13.0/voltage;
            koeff=Math.pow(koeff,1.25);
            //прописать движения здесь:
            //-++- vpered
            //---- protiv chasovoi
            //-+-+ vpravo
            double time1;
            double time2;
            int rl=0;
            if(pologenieskystone=="left")
            {
                rl=1;
                time1=getRuntime();
                time2=getRuntime();
                while(!isStopRequested()&&time2-time1<1.06){
                    setMotorsPowerback(-0.35*koeff,0.35*koeff,0.35*koeff,-0.35*koeff,angles, imu, 0);
                    time2=getRuntime();
                }
                setMotorsPower(0,0,0,0);
                sleep(100);
                time1=getRuntime();
                time2=getRuntime();
                while(!isStopRequested()&&time2-time1<0.8){
                    setMotorsPowerleft(0.3*koeff,-0.3*koeff,0.3*koeff,-0.3*koeff,angles, imu, 0);
                    time2=getRuntime();
                }
                setMotorsPower(0,0,0,0);
                sleep(100);
                goforskystoneopencv(koeff);
                /*setMotorsPowerTimed(-0.35*koeff,0.35*koeff,0.35*koeff,-0.35*koeff,1060);//vpered
                sleep(200);
                setMotorsPowerTimed(0.2*koeff,-0.2*koeff,0.2*koeff,-0.2*koeff,550);//vlevo
                sleep(200);
                goforskystoneopencv(koeff);*/

            }
            if(pologenieskystone=="middle")
            {
                time1=getRuntime();
                time2=getRuntime();
                while(!isStopRequested()&&time2-time1<1.06){
                    setMotorsPowerback(-0.35*koeff,0.35*koeff,0.35*koeff,-0.35*koeff,angles, imu, 0);
                    time2=getRuntime();
                }
                setMotorsPower(0,0,0,0);
                sleep(100);
                time1=getRuntime();
                time2=getRuntime();
                while(!isStopRequested()&&time2-time1<0.2){
                    setMotorsPowerleft(-0.2*koeff,0.2*koeff,-0.2*koeff,0.2*koeff,angles, imu, 0);
                    time2=getRuntime();
                }
                setMotorsPower(0,0,0,0);
                sleep(100);
                goforskystoneopencv(koeff);
                /*
                setMotorsPowerTimed(-0.35*koeff,0.35*koeff,0.35*koeff,-0.35*koeff,1060);//vpered
                sleep(200);
                setMotorsPowerTimed(-0.2*koeff,0.2*koeff,-0.2*koeff,0.2*koeff,200);//vpravo
                sleep(200);
                goforskystoneopencv(koeff);*/
            }
            int rv=0;
            if(pologenieskystone=="right")
            {
                time1=getRuntime();
                time2=getRuntime();
                while(!isStopRequested()&&time2-time1<1.06){
                    setMotorsPowerback(-0.35*koeff,0.35*koeff,0.35*koeff,-0.35*koeff,angles, imu, 0);
                    time2=getRuntime();
                }
                setMotorsPower(0,0,0,0);
                sleep(100);
                time1=getRuntime();
                time2=getRuntime();
                while(!isStopRequested()&&time2-time1<0.91){
                    setMotorsPowerright(-0.3*koeff,0.3*koeff,-0.3*koeff,0.3*koeff,angles, imu, 0);
                    time2=getRuntime();
                }
                setMotorsPower(0,0,0,0);
                sleep(100);
                goforskystoneopencv(koeff);
                rv=1;
                /*
                setMotorsPowerTimed(-0.35*koeff,0.35*koeff,0.35*koeff,-0.35*koeff,1060);//vpered
                sleep(200);
                setMotorsPowerTimed(-0.2*koeff,0.2*koeff,-0.2*koeff,0.2*koeff,650);//vpravo
                sleep(200);
                goforskystoneopencv(koeff);*/
            }
            //setMotorsPowerTimed(0.35*koeff,-0.35*koeff,-0.35*koeff,0.35*koeff,300);//nazad
            //sleep(200);

            angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            while(!isStopRequested()&&angles.firstAngle<82){
                setMotorsPower(-0.3*koeff,-0.3*koeff,-0.3*koeff,-0.3*koeff);
                angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }
            setMotorsPower(0,0,0,0);
            //setMotorsPowerTimed(-0.3 * koeff, -0.3 * koeff, -0.3 * koeff, -0.3 * koeff, 950);//povorot
            //sleep(100);
            setMotorsPowerTimed(-0.4*koeff,0.4*koeff,0.4*koeff,-0.4*koeff,200);//vpered

            time1=getRuntime();
            time2=getRuntime();
            while(!isStopRequested()&&time2-time1<0.6*pologenieskystone1+1+rv*0.21-rl*0.1){
                setMotorsPowerforvard(-0.6*koeff,0.6*koeff,0.6*koeff,-0.6*koeff,angles, imu, 90);
                time2=getRuntime();
            }
            setMotorsPowerTimed(-0.4*koeff,0.4*koeff,0.4*koeff,-0.4*koeff,100);
            sleep(100);

            angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            while(!isStopRequested()&&angles.firstAngle>20){
                setMotorsPower(0.3*koeff,0.3*koeff,0.3*koeff,0.3*koeff);
                angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }
            setMotorsPower(0,0,0,0);
            //setMotorsPowerTimed(0.3 * koeff, 0.3 * koeff, 0.3 * koeff, 0.3 * koeff, 945);//
            //sleep(100);
            setMotorsPowerTimed(-0.3 * koeff, 0.3 * koeff, 0.3 * koeff, -0.3 * koeff, 500);//
            sleep(100);
            s5Shovel.setPosition(0.45);
            sleep(500);

            time1=getRuntime();
            time2=getRuntime();
            while(!isStopRequested()&&time2-time1<0.7){
                setMotorsPowerforvard(0.3*koeff,-0.3*koeff,-0.3*koeff,0.3*koeff,angles, imu,0 );
                time2=getRuntime();
            }
            //setMotorsPowerTimed(0.3 * koeff, -0.3 * koeff, -0.3 * koeff, 0.3 * koeff, 700);//
            //sleep(100);
            while(!isStopRequested()&&angles.firstAngle<82){
                setMotorsPower(-0.45*koeff,-0.45*koeff,-0.45*koeff,-0.45*koeff);
                angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }
            setMotorsPower(0,0,0,0);
            //setMotorsPowerTimed(-0.3 * koeff, -0.3 * koeff, -0.3 * koeff, -0.3 * koeff, 940);//povorot
            //sleep(100);
            setMotorsPowerTimed(-0.3 * koeff, 0.3 * koeff, 0.3 * koeff, -0.3 * koeff, 800);//
            sleep(100);
            s5Shovel.setPosition(0);
            sleep(200);
            //setMotorsPower(0,0,0,0);
            //sleep(200);
            //setMotorsPowerTimed(-0.8*koeff,0.8*koeff,0.8*koeff,-0.8*koeff,600*pologenieskystone1+500);//vpered

            otpustiskystone();

            setMotorsPowerTimed(0.4*koeff,-0.4*koeff,-0.4*koeff,0.4*koeff,400);//nazad
            time1=getRuntime();
            time2=getRuntime();
            while(!isStopRequested()&&time2-time1<0.6*pologenieskystone1+0.8){
                setMotorsPowerback(0.8*koeff,-0.8*koeff,-0.8*koeff,0.8*koeff,angles, imu, 90);
                time2=getRuntime();
            }
            //setMotorsPower(0,0,0,0);
            //sleep(200);
            //setMotorsPowerTimed(0.8*koeff,-0.8*koeff,-0.8*koeff,0.8*koeff,600*pologenieskystone1+900);//nazad1
            setMotorsPowerTimed(0.4*koeff,-0.4*koeff,-0.4*koeff,0.4*koeff,100);//nazad
            sleep(100);
            /*while(DistanceSensor_back.getDistance(DistanceUnit.CM)>30){
                setMotorsPowerback(0.3*koeff,-0.3*koeff,-0.3*koeff,0.3*koeff,angles, imu, 90);
            }
            setMotorsPower(0,0,0,0);*/
            /*int w2=1;
            while(w2==1){
                setMotorsPower(0.2,-0.2,0.26,-0.24);
                if(l==valLeft&&m==valMid&&r==valRight){
                    w2=0;
                    setMotorsPower(0,0,0,0);
                }
            }
            setMotorsPower(0,0,0,0);
            // setMotorsPowerTimed(0.2*koeff,-0.2*koeff,0.24*koeff,-0.22*koeff,400);//vpravo
            setMotorsPowerTimed(0.35,-0.35,0,0,1400);
            setMotorsPowerTimed(0.2*koeff,-0.2*koeff,0.2*koeff,-0.2*koeff,800);//vpravo
            sleep(200);*/

            setMotorsPowerTimed(0.3 * koeff, 0.3 * koeff, 0.3 * koeff, 0.3 * koeff, 930);//povorot
            sleep(100);

            if(pologenieskystone=="left")
            {
                /*
                time1=getRuntime();
                time2=getRuntime();
                while(!isStopRequested()&&time2-time1<0.85){
                    setMotorsPowerleft(0.4*koeff,-0.4*koeff,0.4*koeff,-0.4*koeff,angles, imu, 0);
                    time2=getRuntime();
                }
                setMotorsPower(0,0,0,0);
                sleep(100);*/
                while(!isStopRequested()&&DistanceSensor_right.getDistance(DistanceUnit.CM)>57){
                    setMotorsPowerright(-0.4*koeff,0.4*koeff,-0.4*koeff,0.4*koeff,angles, imu, 0);
                }
                setMotorsPower(0,0,0,0);
                sleep(100);
                while(DistanceSensor_forward.getDistance(DistanceUnit.CM)>15){
                    setMotorsPower(-0.25*koeff,0.25*koeff,0.25*koeff,-0.25*koeff);//
                    //sleep(100);
                }
                setMotorsPower(0,0,0,0);
                goforskystoneopencv(koeff);
                /*
                setMotorsPowerTimed(0.4*koeff,-0.4*koeff,0.4*koeff,-0.4*koeff,850);//vlevo
                sleep(200);
                goforskystoneopencv(koeff);*/
            }
            if(pologenieskystone=="middle")
            {
                /*
                time1=getRuntime();
                time2=getRuntime();
                while(!isStopRequested()&&time2-time1<0.5){
                    setMotorsPowerleft(0.4*koeff,-0.4*koeff,0.4*koeff,-0.4*koeff,angles, imu, 0);
                    time2=getRuntime();
                }
                setMotorsPower(0,0,0,0);
                sleep(100);*/
                while(!isStopRequested()&&DistanceSensor_right.getDistance(DistanceUnit.CM)>30){
                    setMotorsPowerright(-0.4*koeff,0.4*koeff,-0.4*koeff,0.4*koeff,angles, imu, 0);
                }
                setMotorsPower(0,0,0,0);
                sleep(100);
                while(!isStopRequested()&&DistanceSensor_forward.getDistance(DistanceUnit.CM)>15){
                    setMotorsPower(-0.25*koeff,0.25*koeff,0.25*koeff,-0.25*koeff);//
                    //sleep(100);
                }
                setMotorsPower(0,0,0,0);
                goforskystoneopencv(koeff);
                /*
                setMotorsPowerTimed(0.4*koeff,-0.4*koeff,0.4*koeff,-0.4*koeff,500);//vlevo
                sleep(200);
                goforskystoneopencv(koeff);*/
            }
            int right=0;
            if(pologenieskystone=="right")
            {
                /*
                time1=getRuntime();
                time2=getRuntime();
                while(!isStopRequested()&&time2-time1<0.25){
                    setMotorsPowerright(0.4*koeff,-0.4*koeff,0.4*koeff,-0.4*koeff,angles, imu, 0);
                    time2=getRuntime();
                }
                setMotorsPower(0,0,0,0);
                sleep(100);*/
                right =1;
                while(!isStopRequested()&&DistanceSensor_right.getDistance(DistanceUnit.CM)>30){
                    setMotorsPowerright(-0.4*koeff,0.4*koeff,-0.4*koeff,0.4*koeff,angles, imu, 0);
                }
                setMotorsPower(0,0,0,0);
                sleep(100);
                while(!isStopRequested()&&DistanceSensor_forward.getDistance(DistanceUnit.CM)>15){
                    setMotorsPower(-0.25*koeff,0.25*koeff,0.25*koeff,-0.25*koeff);//
                    //sleep(100);
                }
                setMotorsPower(0,0,0,0);
                s4Kicker.setPosition(1);
                sleep(500);
                while(!isStopRequested()&&angles.firstAngle>-20){
                    setMotorsPower(0.35*koeff,0.35*koeff,0.35*koeff,0.35*koeff);
                    angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                }
                setMotorsPower(0,0,0,0);
                goforskystoneopencvright(koeff);
                /*
                setMotorsPowerTimed(0.4*koeff,-0.4*koeff,0.4*koeff,-0.4*koeff,250);//vlevo
                sleep(200);
                goforskystoneopencv(koeff);*/
            }
            //setMotorsPowerTimed(-0.3 * koeff, -0.3 * koeff, -0.3 * koeff, -0.3 * koeff, 940+right*100);//povorot
            //sleep(100);
            angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            while(!isStopRequested()&&angles.firstAngle<82){
                setMotorsPower(-0.35*koeff,-0.35*koeff,-0.35*koeff,-0.35*koeff);
                angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }
            setMotorsPower(0,0,0,0);

            setMotorsPowerTimed(-0.4*koeff,0.4*koeff,0.4*koeff,-0.4*koeff,300);//vpered

            time1=getRuntime();
            time2=getRuntime();
            while(!isStopRequested()&&time2-time1<0.6*pologenieskystone1+1.5+rv*0.1){
                setMotorsPowerforvard(-0.6*koeff,0.6*koeff,0.6*koeff,-0.6*koeff,angles, imu, 90);
                time2=getRuntime();
            }
            //setMotorsPower(0,0,0,0);
            //sleep(200);

            //setMotorsPowerTimed(-0.8*koeff,0.8*koeff,0.8*koeff,-0.8*koeff,600*pologenieskystone1+800);//vpered
            setMotorsPowerTimed(-0.4*koeff,0.4*koeff,0.4*koeff,-0.4*koeff,100);
            sleep(100);
            otpustiskystone();
            while(!isStopRequested()&&DistanceSensor_left.getDistance(DistanceUnit.CM)<65){
                setMotorsPowerright(-0.4*koeff,0.4*koeff,-0.4*koeff,0.4*koeff,angles,imu,90);//
                //sleep(100);
            }
            setMotorsPower(0,0,0,0);
            //while(DistanceSensor_right.getDistance((DistanceUnit.CM))>20)
            time1=getRuntime();
            time2=getRuntime();
            while(!isStopRequested()&&time2-time1<0.85){
                setMotorsPowerforvard(0.8*koeff,-0.8*koeff,-0.8*koeff,0.8*koeff,angles, imu, 90);
                time2=getRuntime();
            }
            setMotorsPower(0,0,0,0);
            //setMotorsPowerTimed(0.8*koeff,-0.8*koeff,-0.8*koeff,0.8*koeff,800);//nazad
            sleep(100);

        }
    }
    //------------------------------------------------------------------------------------------------------------------
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
            double[] pixMid = thresholdMat.get((int)(input.rows()* midPos[0]), (int)(input.cols()* midPos[1]));//gets value at circle
            valMid = (int)pixMid[0];

            double[] pixLeft = thresholdMat.get((int)(input.rows()* leftPos[0]), (int)(input.cols()* leftPos[1]));//gets value at circle
            valLeft = (int)pixLeft[0];

            double[] pixRight = thresholdMat.get((int)(input.rows()* rightPos[0]), (int)(input.cols()* rightPos[1]));//gets value at circle
            valRight = (int)pixRight[0];

            //create three points
            Point pointMid = new Point((int)(input.cols()* midPos[1]), (int)(input.rows()* midPos[0]));
            Point pointLeft = new Point((int)(input.cols()* leftPos[1]), (int)(input.rows()* leftPos[0]));
            Point pointRight = new Point((int)(input.cols()* rightPos[1]), (int)(input.rows()* rightPos[0]));

            //draw circles on those points
            Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointLeft,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols()*(leftPos[1]-rectHeight/2),
                            input.rows()*(leftPos[0]-rectWidth/2)),
                    new Point(
                            input.cols()*(leftPos[1]+rectHeight/2),
                            input.rows()*(leftPos[0]+rectWidth/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols()*(midPos[1]-rectHeight/2),
                            input.rows()*(midPos[0]-rectWidth/2)),
                    new Point(
                            input.cols()*(midPos[1]+rectHeight/2),
                            input.rows()*(midPos[0]+rectWidth/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols()*(rightPos[1]-rectHeight/2),
                            input.rows()*(rightPos[0]-rectWidth/2)),
                    new Point(
                            input.cols()*(rightPos[1]+rectHeight/2),
                            input.rows()*(rightPos[0]+rectWidth/2)),
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
