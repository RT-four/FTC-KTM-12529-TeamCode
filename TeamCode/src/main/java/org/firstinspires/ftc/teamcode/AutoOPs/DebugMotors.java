package org.firstinspires.ftc.teamcode.AutoOPs;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.robot;

@Autonomous(name= "DebugMotors", group="AutoOP")
public class DebugMotors extends robot {
    private ElapsedTime runtime = new ElapsedTime();
    private int time = 3000; //One way rotation time (ms)

    @Override
    public void runOpMode() {
        initHW(hardwareMap);
        telemetry.update();

        waitForStart();
        {

            runtime.reset();

            double voltage = BatteryVoltage();
            double koeff = 13.0 / voltage;
            koeff = Math.pow(koeff, 1.25);

            setMotorsPowerTimed(0.4 * koeff, 0.0 * koeff, 0.0 * koeff, 0.0 * koeff, time);
            setMotorsPowerTimed(-0.4 * koeff, 0.0 * koeff, 0.0 * koeff, 0.0 * koeff, time);
            setMotorsPowerTimed(0.0 * koeff, 0.4 * koeff, 0.0 * koeff, 0.0 * koeff, time);
            setMotorsPowerTimed(0.0 * koeff, -0.4 * koeff, 0.0 * koeff, 0.0 * koeff, time);
            setMotorsPowerTimed(0.0 * koeff, 0.0 * koeff, 0.4 * koeff, 0.0 * koeff, time);
            setMotorsPowerTimed(0.0 * koeff, 0.0 * koeff, -0.4 * koeff, 0.0 * koeff, time);
            setMotorsPowerTimed(0.0 * koeff, 0.0 * koeff, 0.0 * koeff, 0.4 * koeff, time);
            setMotorsPowerTimed(0.0 * koeff, 0.0 * koeff, 0.0 * koeff, -0.4 * koeff, time);
            telemetry.addData("Motors:", "m1Drive (%.2f), m2Drive (%.1f), m3Drive (%.2f), m4Drive (%.2f)", m1Drive, m2Drive, m3Drive, m4Drive);
            telemetry.update();
        }
    }
}
