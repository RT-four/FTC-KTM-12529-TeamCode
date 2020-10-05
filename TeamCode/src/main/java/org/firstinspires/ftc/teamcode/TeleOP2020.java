/*
This program was written by the FTC KTM #12529 team at the Polytechnic University in 2020.

This code is written for managed mode

Our team wishes you all the best for the upcoming tournament.
All versions of the code starting from 2020 you can see here: https://github.com/RT-four/FTC-KTM-12529-TeamCode

Directed by RT-4(Philipp Vasiliev) and Dafter(Daniil Simonovsky)
*/
package org.firstinspires.ftc.teamcode;

import android.os.CountDownTimer;
import org.firstinspires.ftc.teamcode.Robot;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
public class TeleOP2020 extends Robot {
        private ElapsedTime runtime = new ElapsedTime();

        // cutting off the error of sliders values
        private double errorCorrection(double input) {
                if (Math.abs(input) < 0.02) {
                        return 0;
                } else {
                        return Math.signum(input) * (0.9 * Math.pow(Math.abs(input), 2) + 0.1); // Conversion to nonlinear dependence
                }
        }

        @Override
        public void runOpMode() {
                initHW(hardwareMap);
                // Wait for the game to start (driver presses PLAY)
                waitForStart();
                telemetry.clear();
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
                        boolean dpadUp      = gamepad1.dpad_up;
                        boolean dpadRight   = gamepad1.dpad_right;
                        boolean dpadLeft    = gamepad1.dpad_up;
                        boolean dpadDown    = gamepad1.dpad_down;

                        triggerLeft = errorCorrection(triggerLeft);
                        triggerRight = errorCorrection(triggerRight);
                        leftStickX  = errorCorrection(leftStickX);
                        leftStickY  = errorCorrection(leftStickY);
                        rotation    = errorCorrection(rotation);

                        m2DrivePower  = (m1DrivePowerfordrivetofoundation2 + m1DrivePowerfordrivetofoundation22 + m1DrivePowerfordrivetofoundation1 + m1DrivePowerfordrivetofoundation11) + rotation - leftStickY - (triggerLeft + triggerRight + leftStickX);
                        m4DrivePower  = (m2DrivePowerfordrivetofoundation2 + m2DrivePowerfordrivetofoundation22 + m2DrivePowerfordrivetofoundation1 + m2DrivePowerfordrivetofoundation11) + rotation + leftStickY - (triggerLeft + triggerRight + leftStickX);
                        m1DrivePower  = (m3DrivePowerfordrivetofoundation2 + m3DrivePowerfordrivetofoundation22 + m3DrivePowerfordrivetofoundation1 + m3DrivePowerfordrivetofoundation11) + rotation + leftStickY + (triggerLeft + triggerRight + leftStickX);
                        m3DrivePower  = (m4DrivePowerfordrivetofoundation2 + m4DrivePowerfordrivetofoundation22 + m4DrivePowerfordrivetofoundation1 + m4DrivePowerfordrivetofoundation11) + rotation - leftStickY + (triggerLeft + triggerRight + leftStickX);
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
                        telemetry.addData("Motors:", "m1Drive (%.2f), m2Drive (%.1f), m3Drive (%.2f), m4Drive (%.2f)", m1Drive.getPower(), m2Drive.getPower(), m3Drive.getPower(), m4Drive.getPower());
                        telemetry.update();
                }
        }
}
