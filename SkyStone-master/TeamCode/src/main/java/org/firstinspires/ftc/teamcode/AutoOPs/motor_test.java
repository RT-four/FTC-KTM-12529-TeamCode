package org.firstinspires.ftc.teamcode.AutoOPs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robot;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "motor_test", group = "AutoOP")
//@Disabled
public class motor_test extends robot {
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
        initHW(hardwareMap);
        //  wow.init(hardwareMap);
        BNO055IMU.Parameters parametrs= new BNO055IMU.Parameters();
        parametrs.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu rev");
        imu.initialize(parametrs);
        runtime.reset();
        while(!opModeIsActive()) {
            angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("1", angles.firstAngle);
            telemetry.addData("2", angles.secondAngle);
            telemetry.addData("3", angles.thirdAngle);
            telemetry.addData("Distance left: ", DistanceSensor_left.getDistance(DistanceUnit.CM));
            telemetry.addData("Distance right: ", DistanceSensor_right.getDistance(DistanceUnit.CM));
            telemetry.addData("Distance back: ", DistanceSensor_back.getDistance(DistanceUnit.CM));
            telemetry.addData("Distance forward: ", DistanceSensor_forward.getDistance(DistanceUnit.CM));
            telemetry.update();
            sleep(100);
        }
        waitForStart();
        {
            int n=0;
            long sec;
            sec=860;
            double voltage=BatteryVoltage();
            double koeff=13.0/voltage;
            koeff=Math.pow(koeff,1.25);
            setMotorsPowerTimed(-0.4*koeff,0.2*koeff,-0.5*koeff,0.25*koeff,10000);//first
            sleep(200);


        }
    }
}
