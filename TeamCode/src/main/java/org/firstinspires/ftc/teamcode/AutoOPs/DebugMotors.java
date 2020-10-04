/*
This program was written by the FTC KTM #12529 team at the Polytechnic University in 2020.

This code is written to debug the operation of motors.

Our team wishes you all the best for the upcoming tournament.
All versions of the code starting from 2020 you can see here: https://github.com/RT-four/FTC-KTM-12529-TeamCode

Directed by RT-4(Philipp Vasiliev) and Dafter(Daniil Simonovsky)
*/
package org.firstinspires.ftc.teamcode.AutoOPs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name= "DebugMotors", group="AutoOP")
public class DebugMotors extends Robot {
    private ElapsedTime runtime = new ElapsedTime();
    private int time = 3000; //One way rotation time (ms)

    @Override
    public void runOpMode() {
        initHW(hardwareMap);
        telemetry.update();

        waitForStart();
        {
            runtime.reset();

            //Voltage regulation depending on the battery charge level
            double voltage = BatteryVoltage();
            double koeff = 13.0 / voltage;
            koeff = Math.pow(koeff, 1.25);

            setMotorsPowerTimedDebug(0.4 * koeff, 0.0 * koeff, 0.0 * koeff, 0.0 * koeff, time);
            setMotorsPowerTimedDebug(-0.4 * koeff, 0.0 * koeff, 0.0 * koeff, 0.0 * koeff, time);
            setMotorsPowerTimedDebug(0.0 * koeff, 0.4 * koeff, 0.0 * koeff, 0.0 * koeff, time);
            setMotorsPowerTimedDebug(0.0 * koeff, -0.4 * koeff, 0.0 * koeff, 0.0 * koeff, time);
            setMotorsPowerTimedDebug(0.0 * koeff, 0.0 * koeff, 0.4 * koeff, 0.0 * koeff, time);
            setMotorsPowerTimedDebug(0.0 * koeff, 0.0 * koeff, -0.4 * koeff, 0.0 * koeff, time);
            setMotorsPowerTimedDebug(0.0 * koeff, 0.0 * koeff, 0.0 * koeff, 0.4 * koeff, time);
            setMotorsPowerTimedDebug(0.0 * koeff, 0.0 * koeff, 0.0 * koeff, -0.4 * koeff, time);
            telemetry.clear();
            telemetry.addLine("Debug completed");
            telemetry.update();
        }
    }
}
