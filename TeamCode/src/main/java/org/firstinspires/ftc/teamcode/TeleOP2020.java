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


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

@TeleOp(name = "KTM TeleOp 2020", group = "Linear Opmode")
public class TeleOP2020 extends Robot {

    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
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
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Set up our telemetry dashboard

        composeTelemetry();
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
            }
        });

        // Wait until we're told to go
        waitForStart();

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        initHW(hardwareMap);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        telemetry.clear();
        runtime.reset();
        double m1DrivePower;
        double m2DrivePower;
        double m3DrivePower;
        double m4DrivePower;

        double rotationCoefficient = 1.7142; // 1.4285

        while (opModeIsActive()) {
            double triggerLeft = 0.7 * gamepad1.left_trigger;
            double triggerRight = 0.7 * -gamepad1.right_trigger;
            double leftStickY = gamepad1.left_stick_y;
            double leftStickX = -gamepad1.left_stick_x;
            double rotation = 0.7 * gamepad1.right_stick_x;
            boolean dpadUp = gamepad1.dpad_up;
            boolean dpadRight = gamepad1.dpad_right;
            boolean dpadLeft = gamepad1.dpad_left;
            boolean dpadDown = gamepad1.dpad_down;

            triggerLeft = errorCorrection(triggerLeft);
            triggerRight = errorCorrection(triggerRight);
            leftStickX = errorCorrection(leftStickX);
            leftStickY = errorCorrection(leftStickY);
            rotation = errorCorrection(rotation);

            m1DrivePower = 0;
            m2DrivePower = 0;
            m3DrivePower = 0;
            m4DrivePower = 0;

            if (dpadUp) {
                m1DrivePower -= 0.2;
                m2DrivePower += 0.2;
                m3DrivePower += 0.2;
                m4DrivePower -= 0.2;
            }
            if (dpadDown) {
                m1DrivePower += 0.2;
                m2DrivePower -= 0.2;
                m3DrivePower -= 0.2;
                m4DrivePower += 0.2;
            }
            if (dpadLeft) {
                m1DrivePower += 0.33;
                m2DrivePower -= 0.33;
                m3DrivePower += 0.33;
                m4DrivePower -= 0.33;
            }
            if (dpadRight) {
                m1DrivePower -= 0.33;
                m2DrivePower += 0.33;
                m3DrivePower -= 0.33;
                m4DrivePower += 0.33;
            }
            m1DrivePower += (rotation * rotationCoefficient) + leftStickY + (triggerLeft + triggerRight + leftStickX);
            m2DrivePower += (rotation * rotationCoefficient) - leftStickY - (triggerLeft + triggerRight + leftStickX);
            m3DrivePower += (rotation * rotationCoefficient) - leftStickY + (triggerLeft + triggerRight + leftStickX);
            m4DrivePower += (rotation * rotationCoefficient) + leftStickY - (triggerLeft + triggerRight + leftStickX);
            double max = Math.max(Math.max(m1DrivePower, m2DrivePower), Math.max(m3DrivePower, m4DrivePower));
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

            telemetry.addData("Status:", "Run Time: " + runtime.toString());
            telemetry.addData("Required voltage on motors:", "m1Drive (%.2f), m2Drive (%.1f), m3Drive (%.2f), m4Drive (%.2f)", m1DrivePower, m2DrivePower, m3DrivePower, m3DrivePower);
            telemetry.addData("Motor voltage:", "m1Drive (%.2f), m2Drive (%.1f), m3Drive (%.2f), m4Drive (%.2f)", m1Drive.getPower(), m2Drive.getPower(), m3Drive.getPower(), m4Drive.getPower());
            telemetry.update();
        }

    }

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.



        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                });

    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}