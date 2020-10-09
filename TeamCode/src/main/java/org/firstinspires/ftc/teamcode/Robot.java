/*
This program was written by the FTC KTM #12529 team at the Polytechnic University in 2020.

This code is written as a class from which the rest must inherit, to gain access to the main functions of the program.

Our team wishes you all the best for the upcoming tournament.
All versions of the code starting from 2020 you can see here: https://github.com/RT-four/FTC-KTM-12529-TeamCode

Directed by RT-4(Philipp Vasiliev) and Dafter(Daniil Simonovsky (VK: https://vk.com/dafter_play))
*/
package org.firstinspires.ftc.teamcode;

// Major imports
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// Hardware imports
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
// Navigation imports (for angels)
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

// Other classes inherit from this class
public abstract class Robot extends LinearOpMode {
    // Variables for motor chassis
    protected DcMotor m1Drive = null;
    protected DcMotor m2Drive = null;
    protected DcMotor m3Drive = null;
    protected DcMotor m4Drive = null;
    // Variables for navigation
    protected BNO055IMU imu;
    protected Orientation angles;
    // Variable for logs
    private String log = "";

    // Initialization of connected devices
    protected void initHW(HardwareMap hardwareMap) throws RuntimeException {
        // Chassis motor settings
        m1Drive = hardwareMap.get(DcMotor.class, "m1 drive");
        m2Drive = hardwareMap.get(DcMotor.class, "m2 drive");
        m3Drive = hardwareMap.get(DcMotor.class, "m3 drive");
        m4Drive = hardwareMap.get(DcMotor.class, "m4 drive");
        m1Drive.setDirection(DcMotor.Direction.FORWARD);
        m2Drive.setDirection(DcMotor.Direction.FORWARD);
        m3Drive.setDirection(DcMotor.Direction.FORWARD);
        m4Drive.setDirection(DcMotor.Direction.FORWARD);
        m1Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m2Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m3Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m4Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Navigation settings (for angels)
        // Settings parameters for imu
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        // Setting imu settings
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // Displaying information about loading HardwereMap
        telemetry.clear();
        telemetry.addLine("HardwareMap initialization complete");
        telemetry.update();
    }

    // Protection against negative voltage values
    protected double BatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    // Stop all chassis motors
    protected void chassisStopMovement() {
        m1Drive.setPower(0);
        m2Drive.setPower(0);
        m3Drive.setPower(0);
        m4Drive.setPower(0);
    }

    // Setting certain values for motors for a while
    protected void setMotorsPowerTimed(double m1_power, double m2_power, double m3_power, double m4_power, long ms) {
        m1Drive.setPower(m1_power);
        m2Drive.setPower(m2_power);
        m3Drive.setPower(m3_power);
        m4Drive.setPower(m4_power);
        sleep(ms);
        chassisStopMovement();
    }

    // Setting certain values for motors for a while with the output of the values in telemetry
    // Warning: possible slight error in time, use only for debugging
    protected void setMotorsPowerTimedDebug(double m1_power, double m2_power, double m3_power, double m4_power, long ms) {
        int time = 0;
        m1Drive.setPower(m1_power);
        m2Drive.setPower(m2_power);
        m3Drive.setPower(m3_power);
        m4Drive.setPower(m4_power);
        while (time <= ms) {
            telemetry.addData("Motors:", "m1Drive (%.2f), m2Drive (%.1f), m3Drive (%.2f), m4Drive (%.2f)", m1Drive.getPower(), m2Drive.getPower(), m3Drive.getPower(), m4Drive.getPower());
            telemetry.update();
            sleep(1);
            time += 1;
        }
        chassisStopMovement();
    }

    // All for logs
    protected void log(String WhatToSave, Double Value) {
        log += WhatToSave + ": " + Value + "\n";
    }
    protected void log(String WhatToSave) {
        log += WhatToSave + "\n";
    }
    protected String printLog() {
        return log;
    }
}
