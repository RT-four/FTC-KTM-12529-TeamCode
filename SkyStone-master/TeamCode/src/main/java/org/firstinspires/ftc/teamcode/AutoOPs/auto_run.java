package org.firstinspires.ftc.teamcode.AutoOPs;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.robot;

@Autonomous(name= "auto_run", group="AutoOP")
//@Disabled//comment out this line before using
public class auto_run extends robot {
private ElapsedTime runtime = new ElapsedTime();

@Override
public void runOpMode() {
        initHW(hardwareMap);
        //  wow.init(hardwareMap);
        s5Shovel.setPosition(0);
        s4Kicker.setPosition(0);
        s3Rotation.setPosition(0);
        telemetry.update();
        sleep(100);
        s3Rotation.setPosition(0.7);

        waitForStart();

        double voltage=BatteryVoltage();
        double koeff=13.0/voltage;

        setMotorsPowerTimed(-0.4*koeff,0.4*koeff,0.4*koeff,-0.4*koeff,300);
        setMotorsPowerTimed(-0.4*koeff,0.4*koeff,-0.4*koeff,0.4*koeff,1600);
        sleep(100);
        sleep(3500);
        sleep(500);


}
}
