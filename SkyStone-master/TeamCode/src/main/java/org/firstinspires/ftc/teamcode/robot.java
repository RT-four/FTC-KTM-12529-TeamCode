package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

public abstract class robot extends LinearOpMode {
    protected static final int LED_CHANNEL = 5;
    protected DcMotor m1Drive = null;
    protected DcMotor m2Drive = null;
    protected DcMotor m3Drive = null;
    protected DcMotor m4Drive = null;
    protected CRServo s1TopClaw = null;
    protected Servo s4Kicker = null;
    protected Servo s3Rotation = null;
    protected Servo s5Shovel = null;
    protected Servo s6RelicClaw = null;
    protected DcMotor m6Intake = null;
    protected DcMotor m5Lift = null;
    protected DcMotor m7relutka=null;
    //protected ColorSensor sensorRGB;
    //protected TouchSensor touchSensor;
    protected Double shininessCoefficient = 1.8;
    protected DistanceSensor DistanceSensor_left;
    protected DistanceSensor DistanceSensor_right;
    protected DistanceSensor DistanceSensor_back;
    protected DistanceSensor DistanceSensor_forward;
    protected float hsvValues[] = {0F, 0F, 0F};
    private String log = "";

/*protected void spusk(){
    setMotorsPowerTimed(0,0,0,0,-0.2,0,2200);
    sleep(1000);
    setMotorsPowerTimed(-0.1,0.1,0.1,-0.1,400);//vpered
    sleep(200);
}*/
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
/*protected void minusgoldmineral(){
    setMotorsPowerTimed(-0.2,0.2,-0.2,0.2,800);//vlevo
    sleep(200);
    setMotorsPowerTimed(0.2,-0.2,0.2,-0.2,800);//vlevo
    sleep(200);
    s5Shovel.setPosition(0);
}
    protected void putBox() {
        setMotorsPowerTimed(0.18, -0.18, -0.18, 0.18, 0, 0, 1200);//РґРІРёР¶РµРЅРёРµ РЅР°Р·Р°Рґ
        setMotorsPowerTimed(-0.2, 0.2, 0.2, -0.2, 0, 0, 300);//РґРІРёР¶РµРЅРёРµ РІРїРµСЂС‘Рґ
        rotateClaw(0);
        sleep(700);
        setMotorsPowerTimed(0.2, -0.2, -0.2, 0.2, 0, 0, 400);//РґРІРёР¶РµРЅРёРµ РІРїРµСЂС‘Рґ
        setMotorsPowerTimed(-0.2, 0.2, 0.2, -0.2, 0, 0, 300);//РґРІРёР¶РµРЅРёРµ РЅР°Р·Р°Рґ
        rotateClaw(0.8);
    }

    // Rotate claw
    protected void rotateClaw(double rotate) { //if rotate true then rotate to  180 . else to 0
        s3Rotation.setPosition(rotate);
    }*/

    protected void setMotorsPower(double D1_power, double D2_power, double D3_power, double D4_power) { //Warning: Р­С‚Р° С„СѓРЅРєС†РёСЏ РІРєР»СЋС‡РёС‚ РјРѕС‚РѕСЂС‹ РЅРѕ, РІС‹РєР»СЋС‡РёС‚СЊ РёС… РЅР°РґРѕ Р±СѓРґРµС‚ РїРѕСЃР»Рµ РІС‹РїРѕР»РЅРµРЅРёСЏ РєР°РєРѕРіРѕ Р»РёР±Рѕ СѓСЃР»РѕРІРёСЏ
        // Send power to wheels
        m1Drive.setPower(D1_power);
        m2Drive.setPower(D2_power);
        m3Drive.setPower(D3_power);
        m4Drive.setPower(D4_power);
    }
    protected void setMotorsPower(double D1_power, double D2_power, double D3_power, double D4_power, Orientation angles, BNO055IMU imu) { //Warning: Р­С‚Р° С„СѓРЅРєС†РёСЏ РІРєР»СЋС‡РёС‚ РјРѕС‚РѕСЂС‹ РЅРѕ, РІС‹РєР»СЋС‡РёС‚СЊ РёС… РЅР°РґРѕ Р±СѓРґРµС‚ РїРѕСЃР»Рµ РІС‹РїРѕР»РЅРµРЅРёСЏ РєР°РєРѕРіРѕ Р»РёР±Рѕ СѓСЃР»РѕРІРёСЏ
        // Send power to wheels
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        m1Drive.setPower(D1_power +angles.firstAngle / 30);
        m2Drive.setPower(D2_power);
        m3Drive.setPower(D3_power+ angles.firstAngle / 30);
        m4Drive.setPower(D4_power);
    }
    protected void setMotorsPowerforvard(double D1_power, double D2_power, double D3_power, double D4_power, Orientation angles, BNO055IMU imu, double angle) { //Warning: Р­С‚Р° С„СѓРЅРєС†РёСЏ РІРєР»СЋС‡РёС‚ РјРѕС‚РѕСЂС‹ РЅРѕ, РІС‹РєР»СЋС‡РёС‚СЊ РёС… РЅР°РґРѕ Р±СѓРґРµС‚ РїРѕСЃР»Рµ РІС‹РїРѕР»РЅРµРЅРёСЏ РєР°РєРѕРіРѕ Р»РёР±Рѕ СѓСЃР»РѕРІРёСЏ
        // Send power to wheels
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        m1Drive.setPower(D1_power );
        m2Drive.setPower(D2_power-(angle-angles.firstAngle )/ 24);
        m3Drive.setPower(D3_power);
        m4Drive.setPower(D4_power-(angle-angles.firstAngle) / 24);
    }
    //проверить обратную
    protected void setMotorsPowerback(double D1_power, double D2_power, double D3_power, double D4_power, Orientation angles, BNO055IMU imu, double angle) { //Warning: Р­С‚Р° С„СѓРЅРєС†РёСЏ РІРєР»СЋС‡РёС‚ РјРѕС‚РѕСЂС‹ РЅРѕ, РІС‹РєР»СЋС‡РёС‚СЊ РёС… РЅР°РґРѕ Р±СѓРґРµС‚ РїРѕСЃР»Рµ РІС‹РїРѕР»РЅРµРЅРёСЏ РєР°РєРѕРіРѕ Р»РёР±Рѕ СѓСЃР»РѕРІРёСЏ
        // Send power to wheels
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        m1Drive.setPower(D1_power - (angle-angles.firstAngle )/ 24);
        m2Drive.setPower(D2_power);
        m3Drive.setPower(D3_power - (angle-angles.firstAngle) / 24);
        m4Drive.setPower(D4_power);
    }
    protected void setMotorsPowerright(double D1_power, double D2_power, double D3_power, double D4_power, Orientation angles, BNO055IMU imu, double angle) { //Warning: Р­С‚Р° С„СѓРЅРєС†РёСЏ РІРєР»СЋС‡РёС‚ РјРѕС‚РѕСЂС‹ РЅРѕ, РІС‹РєР»СЋС‡РёС‚СЊ РёС… РЅР°РґРѕ Р±СѓРґРµС‚ РїРѕСЃР»Рµ РІС‹РїРѕР»РЅРµРЅРёСЏ РєР°РєРѕРіРѕ Р»РёР±Рѕ СѓСЃР»РѕРІРёСЏ
        // Send power to wheels
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        m1Drive.setPower(D1_power - (angle-angles.firstAngle )/ 30);
        m2Drive.setPower(D2_power);
        m3Drive.setPower(D3_power - (angle-angles.firstAngle) / 30);
        m4Drive.setPower(D4_power);
    }
    protected void setMotorsPowerleft(double D1_power, double D2_power, double D3_power, double D4_power, Orientation angles, BNO055IMU imu, double angle) { //Warning: Р­С‚Р° С„СѓРЅРєС†РёСЏ РІРєР»СЋС‡РёС‚ РјРѕС‚РѕСЂС‹ РЅРѕ, РІС‹РєР»СЋС‡РёС‚СЊ РёС… РЅР°РґРѕ Р±СѓРґРµС‚ РїРѕСЃР»Рµ РІС‹РїРѕР»РЅРµРЅРёСЏ РєР°РєРѕРіРѕ Р»РёР±Рѕ СѓСЃР»РѕРІРёСЏ
        // Send power to wheels
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        m1Drive.setPower(D1_power - (angle-angles.firstAngle )/ 30);
        m2Drive.setPower(D2_power);
        m3Drive.setPower(D3_power - (angle-angles.firstAngle) / 30);
        m4Drive.setPower(D4_power);
    }
    /*protected void setMotorsPower(double D1_power, double D2_power, double D3_power, double D4_power, Orientation angles, BNO055IMU imu, double r) { //Warning: Р­С‚Р° С„СѓРЅРєС†РёСЏ РІРєР»СЋС‡РёС‚ РјРѕС‚РѕСЂС‹ РЅРѕ, РІС‹РєР»СЋС‡РёС‚СЊ РёС… РЅР°РґРѕ Р±СѓРґРµС‚ РїРѕСЃР»Рµ РІС‹РїРѕР»РЅРµРЅРёСЏ РєР°РєРѕРіРѕ Р»РёР±Рѕ СѓСЃР»РѕРІРёСЏ
        // Send power to wheels
        DistanceSensor_left.getDistance(DistanceUnit.CM);//дописать с дистанцией которая получается из аргументов функции
        if(DistanceSensor_left.getDistance(DistanceUnit.CM)<r-2) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            m1Drive.setPower(D1_power + angles.firstAngle / 30);
            m2Drive.setPower(D2_power+(r-DistanceSensor_left.getDistance(DistanceUnit.CM))/150);
            m3Drive.setPower(D3_power-(r-DistanceSensor_left.getDistance(DistanceUnit.CM))/150);
            m4Drive.setPower(D4_power);
        }
        if(DistanceSensor_left.getDistance(DistanceUnit.CM)>r+2) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            m1Drive.setPower(D1_power + angles.firstAngle / 30);
            m2Drive.setPower(D2_power+(r-DistanceSensor_left.getDistance(DistanceUnit.CM))/150);
            m3Drive.setPower(D3_power-(r-DistanceSensor_left.getDistance(DistanceUnit.CM))/150);
            m4Drive.setPower(D4_power);
        }
        if(DistanceSensor_left.getDistance(DistanceUnit.CM)<(r+2)&&DistanceSensor_left.getDistance(DistanceUnit.CM)>(r-2)){
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            m1Drive.setPower(D1_power + angles.firstAngle / 30);
            m2Drive.setPower(D2_power);
            m3Drive.setPower(D3_power);
            m4Drive.setPower(D4_power);
        }
    }*/
    protected void setMotorsPowerTimed(double m1_power, double m2_power, double m3_power, double m4_power, double m5_power, double m6_power, long ms) {
        m1Drive.setPower(m1_power);
        m2Drive.setPower(m2_power);
        m3Drive.setPower(m3_power);
        m4Drive.setPower(m4_power);
        m5Lift.setPower(m5_power);
        m6Intake.setPower(m6_power);
        sleep(ms);
        chassisStopMovement();
    }
    protected void setMotorsPowerTimed(double m1_power, double m2_power, double m3_power, double m4_power, long ms, Orientation angles, BNO055IMU imu) {
        angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        m1Drive.setPower(m1_power);
        m2Drive.setPower(m2_power);
        m3Drive.setPower(m3_power);
        m4Drive.setPower(m4_power);
        sleep(ms);
        chassisStopMovement();
    }
    protected void setMotorsPowerTimed(double m1_power, double m2_power, double m3_power, double m4_power, long ms) {
        m1Drive.setPower(m1_power);
        m2Drive.setPower(m2_power);
        m3Drive.setPower(m3_power);
        m4Drive.setPower(m4_power);
        sleep(ms);
        chassisStopMovement();
    }
    protected void chassisStopMovement() {
        m1Drive.setPower(0);
        m2Drive.setPower(0);
        m3Drive.setPower(0);
        m4Drive.setPower(0);
        m5Lift.setPower(0);
        m6Intake.setPower(0);
    }

    protected void log(String WhatToSave, Double Value) {
        log += WhatToSave + ": " + Value + "\n";
    }

    protected void log(String WhatToSave) {
        log += WhatToSave + "\n";
    }

    protected String printLog() {
        return log;
    }
    /*protected String getColor() {

        double[] hue_arr = new double[5];
        //РґР»СЏ С‚РѕС‡РЅРѕСЃС‚Рё 4 РёР·РјРµСЂРµРЅРёСЏ
        for (int j = 0; j < 4; j++) {
            // convert the RGB values to HSV values.
            telemetry.addData("Blue", sensorRGB.blue());
            telemetry.addData("Red", sensorRGB.red());
            telemetry.update();
            sleep(500);
            Color.RGBToHSV((sensorRGB.red() * 255) / 800, (sensorRGB.green() * 255) / 800, (sensorRGB.blue() * 255) / 800, hsvValues);
            double hue = hsvValues[0];
            hue_arr[j] = hue;
        }

        //РќР°С…РѕРґРёРј СЃСЂРµРґРЅРµРµ Р°СЂРёС„РјРµС‚РёС‡РµСЃРєРѕРµ

        double hue_sr = 0;
        for (int j = 0; j < 4; j++) {
            hue_sr += hue_arr[j];
        }
        hue_sr = hue_sr / 4;
        //
        if (hue_sr > 90 && hue_sr < 310) {
            return "Blue";
        } else {
            return "Red";
        }
    }*/

    protected void initHW(HardwareMap hardwMap) throws RuntimeException {
        m1Drive = hardwMap.get(DcMotor.class, "m1 drive");
        m2Drive = hardwMap.get(DcMotor.class, "m2 drive");
        m3Drive = hardwMap.get(DcMotor.class, "m3 drive");
        m4Drive = hardwMap.get(DcMotor.class, "m4 drive");
        //s1TopClaw = hardwMap.get(CRServo.class, "s1 top claw");
        s4Kicker = hardwMap.get(Servo.class, "s4 kick");
        //odsSensor = hardwMap.get(OpticalDistanceSensor.class, "sensor_ods");
        s3Rotation = hardwMap.get(Servo.class, "s3 rotation");
        s5Shovel = hardwMap.get(Servo.class, "s5 shovel");
        s6RelicClaw = hardwMap.get(Servo.class, "s6 relic claw");
        m6Intake = hardwMap.get(DcMotor.class, "m6 intake");
        //sensorRGB = hardwMap.get(ColorSensor.class, "sensor_color");
        m5Lift = hardwareMap.get(DcMotor.class, "m5 lift");
        m7relutka = hardwareMap.get(DcMotor.class, "m7 rul");
        //touchSensor = hardwMap.get(TouchSensor.class,"sensor touch");
        DistanceSensor_left = hardwMap.get(DistanceSensor.class,"dist left");
        DistanceSensor_right = hardwMap.get(DistanceSensor.class,"dist right");
        DistanceSensor_back = hardwMap.get(DistanceSensor.class,"dist back");
        DistanceSensor_forward = hardwMap.get(DistanceSensor.class,"dist forward");
        m1Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m2Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m3Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m4Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m5Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m6Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    protected int getRelic(VuforiaTrackable relicTemplate) {
        RelicRecoveryVuMark vuMark;
        for (int tick = 0; tick < 4000; tick += 10) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            telemetry.addData("Vumark", vuMark);
            telemetry.update();
            if (vuMark == RelicRecoveryVuMark.UNKNOWN) {
                sleep(10);
            } else {
                if (vuMark == RelicRecoveryVuMark.LEFT) {
                    return 1;
                }
                if (vuMark == RelicRecoveryVuMark.CENTER) {
                    return 2;
                }
                if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    return 3;
                }
            }
        }
        return 99999;
    }
    protected int getPicture(VuforiaTrackable picture) {
        RelicRecoveryVuMark PPicture;
        for (int tick = 0; tick < 4000; tick += 10) {
            PPicture = RelicRecoveryVuMark.from(picture);
            telemetry.addData("Picture", PPicture);
            telemetry.update();
            if (PPicture == RelicRecoveryVuMark.UNKNOWN) {
                sleep(10);
            } else {
                if (PPicture == RelicRecoveryVuMark.LEFT) {
                    return 1;
                }
                if (PPicture == RelicRecoveryVuMark.CENTER) {
                    return 2;
                }
                if (PPicture == RelicRecoveryVuMark.RIGHT) {
                    return 3;
                }
            }
        }
        return 99999;
    }
   /* protected void goForMoreBoxes() {
        m5Lift.setPower(0);
        s5Shovel.setPosition(1);
        s3Rotation.setPosition(0.8);
        setMotorsPowerTimed(-1, 0.85, 0.9, -0.9, 0, 0, 800);

// collecting glyphs
        s5Shovel.setPosition(0.5);
        sleep(350);
        setMotorsPower(0, 0, 0, 0);
        sleep(350);
        // Moving back
        setMotorsPower(0.18, -0.22, -0.18, 0.18);
        s5Shovel.setPosition(0.6);
        sleep(200);
        s5Shovel.setPosition(0.2);
        sleep(200);
        setMotorsPower(0, 0, 0, 0);
        sleep(200);
        s5Shovel.setPosition(0.8);
        sleep(500);
        s5Shovel.setPosition(0);
        sleep(580);
//lifting up

        m5Lift.setPower(0.8);
        sleep(500);
        m5Lift.setPower(0);
        setMotorsPower(0.17, -0.2, -0.18, 0.18);
        sleep(1500);
        chassisStopMovement();
        sleep(200);

        putBox();
        //for safety shovel down
        s5Shovel.setPosition(0.9);
        chassisStopMovement();
        m5Lift.setPower(-0.3);
        sleep(500);
        m5Lift.setPower(0);
        s3Rotation.setPosition(0.8);

    }*/
    protected void goforskystone(double koeff, int vpered){
        setMotorsPowerTimed(0.2*koeff,-0.2*koeff,-0.2*koeff,0.2*koeff,vpered);//vpered
        sleep(200);
        s4Kicker.setPosition(1);
        sleep(1000);
        setMotorsPowerTimed(-0.2*koeff,0.2*koeff,0.2*koeff,-0.2*koeff,250);//nazad
        sleep(200);
        s3Rotation.setPosition(0);
        sleep(1000);
        s4Kicker.setPosition(0);
        sleep(1000);
    }
    protected void goforskystoneopencv(double koeff){
        s4Kicker.setPosition(1);
        sleep(500);
        setMotorsPowerTimed(0.2*koeff,-0.2*koeff,-0.2*koeff,0.2*koeff,250);//nazad
        sleep(100);
        s3Rotation.setPosition(0);
        sleep(500);
        s4Kicker.setPosition(0);
        sleep(700);
    }
    protected void goforskystoneopencvright(double koeff){
        setMotorsPowerTimed(0.2*koeff,-0.2*koeff,-0.2*koeff,0.2*koeff,250);//nazad
        sleep(100);
        s3Rotation.setPosition(0);
        sleep(500);
        s4Kicker.setPosition(0);
        sleep(700);
    }
    protected void otpustiskystone(){
        s4Kicker.setPosition(1);
        sleep(200);
        s3Rotation.setPosition(0.7);
        sleep(200);
        s4Kicker.setPosition(0);
        sleep(200);
    }
    @Deprecated
    protected void goToCryptoBoxRED(double fieldColorSR, ElapsedTime runtime) {
        assert true;
    }

    @Deprecated
    protected void goToCryptoBoxBLUE(double fieldColorSR, ElapsedTime runtime) {
        assert true;
    }
}

