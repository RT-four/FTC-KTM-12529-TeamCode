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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot;


@Autonomous(name = "auto22", group = "AutoOP")
//@Disabled
public class auto22 extends robot {
    DeviceInterfaceModule cdim;
    /**
     * {@link #} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
// robot wow= new robot();

    @Override
    public void runOpMode() {
        initHW(hardwareMap);
      //  wow.init(hardwareMap);
        s5Shovel.setPosition(0);
        s4Kicker.setPosition(0);
        s3Rotation.setPosition(0);
        runtime.reset();

        waitForStart();
        {
            int n=0;
            double voltage=BatteryVoltage();
            double koeff=13.0/voltage;
            koeff=Math.pow(koeff,1.25);
            setMotorsPowerTimed(-0.3*koeff,0.3*koeff,0.5*koeff,-0.5*koeff,1000);//vpered
            //setMotorsPowerTimed(0.3*koeff,-0.3*koeff,-0.3*koeff,0.3*koeff,50);//nazad
            sleep(200);

        }
    }
}
