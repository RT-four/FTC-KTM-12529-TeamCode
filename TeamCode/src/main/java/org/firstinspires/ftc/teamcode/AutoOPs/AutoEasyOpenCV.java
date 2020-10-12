/*
This program was written by the FTC KTM #12529 team at the Polytechnic University in 2020.

This autonomy is the main one. She processes the number of rings and travels along a given trajectory

Our team wishes you all the best for the upcoming tournament.
All versions of the code starting from 2020 you can see here: https://github.com/RT-four/FTC-KTM-12529-TeamCode

Directed by RT-4(Philipp Vasiliev) and Dafter(Daniil Simonovsky (VK: https://vk.com/dafter_play))
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

@Autonomous(name = "AutoEasyOpenCV", group = "AutoOP")
public class AutoEasyOpenCV extends Robot {
    private ElapsedTime runtime = new ElapsedTime();
    OpenCvInternalCamera phoneCam;
    EasyOpenCVVision pipeline;

    @Override
    public void runOpMode() {
        initHW(hardwareMap);

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

            //Voltage regulation depending on the battery charge level
            double voltage = BatteryVoltage();
            double koeff = 13.0 / voltage;
            koeff = Math.pow(koeff, 1.25);
            // The choice of the direction of movement depending on the number of rings
            if (pipeline.position == EasyOpenCVVision.RingPosition.FOUR) {
                setMotorsPowerCorrected(-0.4 * koeff, 0.4 * koeff, 0.4 * koeff, -0.4 * koeff, (short)0, 5000);
            }
            if (pipeline.position == EasyOpenCVVision.RingPosition.ONE) {
                setMotorsPowerCorrected(-0.4 * koeff, 0.4 * koeff, 0.4 * koeff, -0.4 * koeff, (short)0, 5000);
//                setMotorsPowerTimed(-0.2 * koeff, 0.2 * koeff, -0.2 * koeff, 0.2 * koeff, 1650);
            }
            if (pipeline.position == EasyOpenCVVision.RingPosition.NONE) {
                setMotorsPowerCorrected(-0.5 * koeff, 0.5 * koeff, 0.5 * koeff, -0.5 * koeff, (short)0, 10000);
//                setMotorsPowerTimed(0.4 * koeff, -0.4 * koeff, 0.4 * koeff, -0.4 * koeff, 1500);
            }
        }

    }
}