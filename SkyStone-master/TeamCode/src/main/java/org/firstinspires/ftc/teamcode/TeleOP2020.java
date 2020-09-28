package org.firstinspires.ftc.teamcode;

import android.os.CountDownTimer;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name = "KTM TeleOp 2020", group = "Linear Opmode")

public class TeleOP2020 extends LinearOpMode {
private static final int LED_CHANNEL = 5;
private ElapsedTime runtime = new ElapsedTime();

//    cutting off the error of sliders values
double errorCorrection(double input) {
        if (Math.abs(input) < 0.02) {
                return null;
        } else {
                return Math.signum(input) * (0.9 * Math.pow(Math.abs(input), 2) + 0.1);
        }
}


void setMotorPowerTimed(DcMotor motor, double power, long milliseconds) {
        motor.setPower(power);
        sleep(milliseconds);
        motor.setPower(0);
}

void setServoPowerTimed(CRServo Crservo, double power, long milliseconds) {
        Crservo.setPower(power);
        sleep(milliseconds);
        Crservo.setPower(0);
}

@Override
public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        DcMotor m1Drive = hardwareMap.get(DcMotor.class, "m1 drive");
        DcMotor m2Drive = hardwareMap.get(DcMotor.class, "m2 drive");
        DcMotor m3Drive = hardwareMap.get(DcMotor.class, "m3 drive");
        DcMotor m4Drive = hardwareMap.get(DcMotor.class, "m4 drive");
        DcMotor m5Lift = hardwareMap.get(DcMotor.class, "m5 lift");
        DcMotor m6Intake = hardwareMap.get(DcMotor.class, "m6 intake");
        DcMotor m7ruletka = hardwareMap.get(DcMotor.class, "m7 rul");
        CRServo s1RelicExtRet = hardwareMap.get(CRServo.class, "s1 top claw");
        Servo s3Rotation = hardwareMap.get(Servo.class, "s3 rotation");
        Servo s4Kicker = hardwareMap.get(Servo.class, "s4 kick");
        Servo s5Shovel = hardwareMap.get(Servo.class, "s5 shovel");
        Servo s6RelicClaw = hardwareMap.get(Servo.class, "s6 relic claw");
        Servo s7RelicArm = hardwareMap.get(Servo.class, "s7 relic arm");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        m1Drive.setDirection(DcMotor.Direction.FORWARD);
        m2Drive.setDirection(DcMotor.Direction.FORWARD);
        m3Drive.setDirection(DcMotor.Direction.FORWARD);
        m4Drive.setDirection(DcMotor.Direction.FORWARD);
        m5Lift.setDirection(DcMotor.Direction.FORWARD);
        m6Intake.setDirection(DcMotor.Direction.FORWARD);
        m1Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m2Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m3Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m4Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m5Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m6Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        double m1DrivePower;
        double m2DrivePower;
        double m3DrivePower;
        double m4DrivePower;
        double m1DrivePowerfordrivetofoundation1  = 0;
        double m2DrivePowerfordrivetofoundation1  = 0;
        double m3DrivePowerfordrivetofoundation1  = 0;
        double m4DrivePowerfordrivetofoundation1  = 0;
        double m1DrivePowerfordrivetofoundation11 = 0;
        double m2DrivePowerfordrivetofoundation11 = 0;
        double m3DrivePowerfordrivetofoundation11 = 0;
        double m4DrivePowerfordrivetofoundation11 = 0;
        double m1DrivePowerfordrivetofoundation2  = 0;
        double m2DrivePowerfordrivetofoundation2  = 0;
        double m3DrivePowerfordrivetofoundation2  = 0;
        double m4DrivePowerfordrivetofoundation2  = 0;
        double m1DrivePowerfordrivetofoundation22 = 0;
        double m2DrivePowerfordrivetofoundation22 = 0;
        double m3DrivePowerfordrivetofoundation22 = 0;
        double m4DrivePowerfordrivetofoundation22 = 0;

        while (opModeIsActive()) {
                double triggerLeft  = 0.7 * gamepad1.left_trigger;
                double triggerRight = 0.7 * -gamepad1.right_trigger;
                double leftStickY   = gamepad1.left_stick_y;
                double leftStickX   = -gamepad1.left_stick_x;
                double rotation     = 0.7 * gamepad1.right_stick_x;
                double dpadUp       = gamepad1.dpad_up;
                double dpadRight    = gamepad1.dpad_right;
                double dpadLeft     = gamepad1.dpad_up;
                double dpadDown     = gamepad1.dpad_down;

                triggerLeft = errorCorrection(triggerLeft);
                riggerRight = errorCorrection(triggerRight);
                leftStickX  = errorCorrection(leftStickX);
                leftStickY  = errorCorrection(leftStickY);
                rotation    = errorCorrection(rotation);

                m2DrivePower  = (m1DrivePowerfordrivetofoundation2 + m1DrivePowerfordrivetofoundation22 + m1DrivePowerfordrivetofoundation1 + m1DrivePowerfordrivetofoundation11) + rotation - leftStickY - (triggerLeft + triggerRight + rotation);
                m4DrivePower  = (m2DrivePowerfordrivetofoundation2 + m2DrivePowerfordrivetofoundation22 + m2DrivePowerfordrivetofoundation1 + m2DrivePowerfordrivetofoundation11) + rotation + leftStickY - (triggerLeft + triggerRight + rotation);
                m1DrivePower  = (m3DrivePowerfordrivetofoundation2 + m3DrivePowerfordrivetofoundation22 + m3DrivePowerfordrivetofoundation1 + m3DrivePowerfordrivetofoundation11) + rotation + leftStickY + (triggerLeft + triggerRight + rotation);
                m3DrivePower  = (m4DrivePowerfordrivetofoundation2 + m4DrivePowerfordrivetofoundation22 + m4DrivePowerfordrivetofoundation1 + m4DrivePowerfordrivetofoundation11) + rotation - leftStickY + (triggerLeft + triggerRight + rotation);
                double max    = Math.max(Math.max(m1DrivePower, m2DrivePower), Math.max(m3DrivePower, m4DrivePower));

                if (max >= 1) {
                        m1Drive.setPower(m1DrivePower / max);
                        m2Drive.setPower(m2DrivePower / max);
                        m3Drive.setPower(m3DrivePower / max);
                        m4Drive.setPower(m4DrivePower / max);
                } else {
                        m1Drive.setPower(m1DrivePower);
                        m2Drive.setPower(m2DrivePower);
                        m3Drive.setPower(m3DrivePower);
                        m4Drive.setPower(m4DrivePower);
                }


                if (dpadUp) {
                        m1DrivePowerfordrivetofoundation11 = 0.2;
                        m2DrivePowerfordrivetofoundation11 = -0.2;
                        m3DrivePowerfordrivetofoundation11 = -0.2;
                        m4DrivePowerfordrivetofoundation11 = 0.2;
                } else {
                        m1DrivePowerfordrivetofoundation11 = 0;
                        m2DrivePowerfordrivetofoundation11 = 0;
                        m3DrivePowerfordrivetofoundation11 = 0;
                        m4DrivePowerfordrivetofoundation11 = 0;
                }
                if (dpadDown) {
                        m1DrivePowerfordrivetofoundation1 = -0.2;
                        m2DrivePowerfordrivetofoundation1 = 0.2;
                        m3DrivePowerfordrivetofoundation1 = 0.2;
                        m4DrivePowerfordrivetofoundation1 = -0.2;
                } else {
                        m1DrivePowerfordrivetofoundation1 = 0;
                        m2DrivePowerfordrivetofoundation1 = 0;
                        m3DrivePowerfordrivetofoundation1 = 0;
                        m4DrivePowerfordrivetofoundation1 = 0;
                }
                if (dpadLeft) {
                        m1DrivePowerfordrivetofoundation2 = -0.33;
                        m2DrivePowerfordrivetofoundation2 = -0.33;
                        m3DrivePowerfordrivetofoundation2 = 0.33;
                        m4DrivePowerfordrivetofoundation2 = 0.33;
                } else {
                        m1DrivePowerfordrivetofoundation2 = 0;
                        m2DrivePowerfordrivetofoundation2 = 0;
                        m3DrivePowerfordrivetofoundation2 = 0;
                        m4DrivePowerfordrivetofoundation2 = 0;
                }
                if (dpadRight) {
                        m1DrivePowerfordrivetofoundation22 = 0.33;
                        m2DrivePowerfordrivetofoundation22 = 0.33;
                        m3DrivePowerfordrivetofoundation22 = -0.33;
                        m4DrivePowerfordrivetofoundation22 = -0.33;
                } else {
                        m1DrivePowerfordrivetofoundation22 = 0;
                        m2DrivePowerfordrivetofoundation22 = 0;
                        m3DrivePowerfordrivetofoundation22 = 0;
                        m4DrivePowerfordrivetofoundation22 = 0;
                }


                telemetry.addData("Status:", "Run Time: " + runtime.toString());
                telemetry.addData("Motors:", "m1Drive (%.2f), m2Drive (%.1f), m3Drive (%.2f), m4Drive (%.2f)", m1DrivePower, m2DrivePower, m3DrivePower, m4DrivePower);
                telemetry.update();
        }
}
}
