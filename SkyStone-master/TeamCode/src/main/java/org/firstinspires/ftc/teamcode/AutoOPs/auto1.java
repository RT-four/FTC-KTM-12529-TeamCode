/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.AutoOPs;

import android.os.CountDownTimer;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robot;

@Autonomous(name = "auto1", group = "AutoOP")
//@Disabled
public class auto1 extends robot {
    //DeviceInterfaceModule cdim;
    /**
     * {@link #} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
// robot wow= new robot();
    BNO055IMU imu;
    Orientation angles;
    @Override
    public void runOpMode() {
        BNO055IMU.Parameters parametrs= new BNO055IMU.Parameters();
        parametrs.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu rev");
        imu.initialize(parametrs);
        initHW(hardwareMap);
      //  wow.init(hardwareMap);
        s5Shovel.setPosition(0);
        s4Kicker.setPosition(0);
        s3Rotation.setPosition(0);
        runtime.reset();
        waitForStart();
        {
            /*CountDownTimer timer= new CountDownTimer(120000, 1000) {
                @Override
                public void onTick(long l) {
                    telemetry.addData("timer", l);
                    telemetry.update();
                }

                @Override
                public void onFinish() {

                }
            };
            timer.start();*/
            int a=0;
            double voltage=BatteryVoltage();
            double koeff=13.0/voltage;
            koeff=Math.pow(koeff,1.25);
            angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double time=getRuntime();
            double timenew=getRuntime()-time;
            /*while(opModeIsActive()) {
                telemetry.addData("time (%.5f)", time);
                telemetry.addData("timenew (%.5f)", timenew);
                //telemetry.addData("timer (%.5f)", timer.toString());
                telemetry.addData("time from runtime (%.5f)", getRuntime());
                telemetry.update();
            }*/
            //getRuntime();
            /*while(DistanceSensor_right.getDistance(DistanceUnit.CM)>30){
                setMotorsPower(-0.4*koeff,0.4*koeff,0.4*koeff,-0.4*koeff);
                a++;
            }
            setMotorsPower(0,0,0,0);
            while(angles.firstAngle<84) {
                setMotorsPower(-0.3 * koeff, -0.3 * koeff, -0.3 * koeff, -0.3 * koeff);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }
            setMotorsPower(0,0,0,0);*/
            while(!isStopRequested()&&angles.firstAngle>-20){
                setMotorsPower(0.35*koeff,0.35*koeff,0.35*koeff,0.35*koeff);
                angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }
            setMotorsPower(0,0,0,0);
            sleep(10000);
            while(!isStopRequested()&&angles.firstAngle<82){
                setMotorsPower(-0.45*koeff,-0.45*koeff,-0.45*koeff,-0.45*koeff);
                angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }
            setMotorsPower(0,0,0,0);
            setMotorsPowerTimed(-0.3 * koeff, -0.3 * koeff, -0.3 * koeff, -0.3 * koeff, 940);//povorot
            sleep(200);
            sleep(5000);
            double time1=getRuntime();
            double time2=getRuntime();
            while(!isStopRequested()&&time2-time1<5){
                setMotorsPowerback(-0.2*koeff,0.2*koeff,0.2*koeff,-0.2*koeff,angles, imu, 90);
                time2=getRuntime();
                telemetry.addData("time1 (%.5f)", time1);
                telemetry.addData("timenew (%.5f)", time2);
                //telemetry.addData("timer (%.5f)", timer.toString());
                telemetry.update();
            }
            sleep(2000);

            telemetry.addData("time1 (%.5f)", time1);
            telemetry.addData("timenew (%.5f)", time2);
            //telemetry.addData("timer (%.5f)", timer.toString());
            telemetry.update();
            setMotorsPower(0,0,0,0);
            sleep(10000);
            //setMotorsPowerTimed(-0.1 * koeff, 0.1 * koeff, 0.1 * koeff, -0.1 * koeff, 10000, angles, imu);
            //setMotorsPowerTimed(-0.3 * koeff, -0.3 * koeff, -0.3 * koeff, -0.3 * koeff, 950);
            angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("a: ", a);
            telemetry.addData("angle: ", angles.firstAngle);
            telemetry.update();
            sleep(10000);


        }
    }
}
