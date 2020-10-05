/*
This program was written by the FTC KTM #12529 team at the Polytechnic University in 2020.

This autonomy is the main one. She processes the number of rings and travels along a given trajectory

Our team wishes you all the best for the upcoming tournament.
All versions of the code starting from 2020 you can see here: https://github.com/RT-four/FTC-KTM-12529-TeamCode

Directed by RT-4(Philipp Vasiliev) and Dafter(Daniil Simonovsky)
*/

package org.firstinspires.ftc.teamcode.AutoOPs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import org.firstinspires.ftc.teamcode.Vision.EasyOpenCVVision;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

@Autonomous(name = "AutoEasyOpenCV", group = "AutoOP")
public class AutoEasyOpenCV extends Robot {
    private ElapsedTime runtime = new ElapsedTime();
    OpenCvInternalCamera phoneCam;
    EasyOpenCVVision pipeline;

    @Override
    public void runOpMode() {
        initHW(hardwareMap);
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)distanceSensorForward;

        // Camera setup
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new EasyOpenCVVision();
        phoneCam.setPipeline(pipeline);
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });
        telemetry.addLine("Phone camera initialized");
        telemetry.update();

        waitForStart();
        {
            telemetry.clear();
            telemetry.addData("Number of rings ", pipeline.position);
            telemetry.update();
//                // generic DistanceSensor methods.
//                telemetry.addData("deviceName",distanceSensorForward.getDeviceName() );
//                telemetry.addData("range", String.format("%.01f m", distanceSensorForward.getDistance(DistanceUnit.METER)));
//
//
//                // Rev2mDistanceSensor specific methods.
//                telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
//                telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));
//
//                telemetry.update();

            //Voltage regulation depending on the battery charge level
            double voltage = BatteryVoltage();
            double koeff = 13.0 / voltage;
            koeff = Math.pow(koeff, 1.25);
            setMotorsPowerTimed(-0.5 * koeff, 0.5 * koeff, 0.5 * koeff, -0.5 * koeff, 1000);
            while ((int)distanceSensorForward.getDistance(DistanceUnit.CM)>(int)50){
                m1Drive.setPower(-0.2);
                m2Drive.setPower(0.2);
                m3Drive.setPower(0.2);
                m4Drive.setPower(-0.2);

                telemetry.addData("Distance to the wall: ", String.format("%.01f cm", distanceSensorForward.getDistance(DistanceUnit.CM)));

                telemetry.update();
            }
            chassisStopMovement();

            // The choice of the direction of movement depending on the number of rings
//            if (pipeline.position == EasyOpenCVVision.RingPosition.FOUR) {
//                setMotorsPowerTimed(-0.4 * koeff, 0.4 * koeff, 0.4 * koeff, -0.4 * koeff, 1600);
//            }
//            if (pipeline.position == EasyOpenCVVision.RingPosition.ONE) {
//                setMotorsPowerTimed(-0.2 * koeff, 0.2 * koeff, -0.2 * koeff, 0.2 * koeff, 1650);
//            }
//            if (pipeline.position == EasyOpenCVVision.RingPosition.NONE) {
//                setMotorsPowerTimed(0.4 * koeff, -0.4 * koeff, 0.4 * koeff, -0.4 * koeff, 1500);
//            }
        }
    }
}