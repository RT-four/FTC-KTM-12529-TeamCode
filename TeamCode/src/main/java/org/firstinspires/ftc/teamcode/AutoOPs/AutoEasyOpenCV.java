/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
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

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new EasyOpenCVVision();
        phoneCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });

        waitForStart();
        {

            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);

            double voltage = BatteryVoltage();
            double koeff = 13.0 / voltage;
            koeff = Math.pow(koeff, 1.25);
            double time1;
            double time2;
            if (pipeline.position == EasyOpenCVVision.RingPosition.FOUR) {
                setMotorsPowerTimed(-0.4 * koeff, 0.4 * koeff, 0.4 * koeff, -0.4 * koeff, 1600);
                telemetry.addData("Detected ring position", pipeline.position);
                telemetry.update();
            }
            if (pipeline.position == EasyOpenCVVision.RingPosition.ONE) {
                setMotorsPowerTimed(-0.2 * koeff, 0.2 * koeff, -0.2 * koeff, 0.2 * koeff, 1650);
                telemetry.addData("Detected ring position", pipeline.position);
                telemetry.update();
            }
            if (pipeline.position == EasyOpenCVVision.RingPosition.NONE) {
                setMotorsPowerTimed(0.4 * koeff, -0.4 * koeff, 0.4 * koeff, -0.4 * koeff, 1500);
                telemetry.addData("Detected ring position", pipeline.position);
                telemetry.update();
            }
            sleep(50);


        }
    }
}