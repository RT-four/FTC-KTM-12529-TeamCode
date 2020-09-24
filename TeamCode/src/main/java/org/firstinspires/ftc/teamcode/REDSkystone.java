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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import pkg3939.Robot3939;
import pkg3939.skystoneDetectorClass;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="REDSkystone", group="Pushbot")
//@Disabled
public class REDSkystone extends LinearOpMode {

    /* Declare OpMode members. */
    Robot3939 robot = new Robot3939();   // Use a Pushbot's hardware

    skystoneDetectorClass detector = new skystoneDetectorClass();
    int[] vals;
    private ElapsedTime     runtime = new ElapsedTime();

    private final double gearRatio = 2/1;//2:1
    private final double ticksPerRev = 537.6 * gearRatio;
    private final double wheelCircumference = 3.1415 * robot.wheelDiameter; //pi * diameter (inches)

    public void rotate(double power, double time) {
        robot.FL.setPower(power);
        robot.FR.setPower(-power);
        robot.RL.setPower(power);
        robot.RR.setPower(-power);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 4:  Stop and close the claw.
        robot.FL.setPower(0);
        robot.FR.setPower(0);
        robot.RL.setPower(0);
        robot.RR.setPower(0);
        sleep(200);

    }

    public void rotateEnc(int targetTicks) {
        double k = 0.0004;

        robot.stopAndResetEncoders();

        robot.useEncoders(true);

        double tickAvg = (int)((Math.abs(robot.FL.getCurrentPosition()) + Math.abs(robot.FR.getCurrentPosition()) + Math.abs(robot.RL.getCurrentPosition()) + Math.abs(robot.RR.getCurrentPosition()))/4);
        double tickDifference = targetTicks - tickAvg;
        double power = Range.clip(k*Math.abs(tickDifference) + 0.2, 0.2, 0.8);

        if(tickDifference < 0)
            power = -power;

        if(opModeIsActive()) {
            robot.RL.setTargetPosition(targetTicks);
            robot.RR.setTargetPosition(-targetTicks);
            robot.FL.setTargetPosition(targetTicks);
            robot.FR.setTargetPosition(-targetTicks);

            robot.RL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.RR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.setAllGivenPower(power);

            double startAngle = robot.getAngle();

            while(robot.RL.isBusy() || robot.RR.isBusy() || robot.FL.isBusy() || robot.FR.isBusy()) {
                //wait till motor finishes

                tickAvg = (int)((Math.abs(robot.FL.getCurrentPosition()) + Math.abs(robot.FR.getCurrentPosition()) + Math.abs(robot.RL.getCurrentPosition()) + Math.abs(robot.RR.getCurrentPosition()))/4);
                tickDifference = targetTicks - tickAvg;
                power = Range.clip(k*Math.abs(tickDifference) + 0.2, 0.2, 1);

                if(tickDifference < 0)
                    power = -power;
//                 double correction = robot.getCorrection(startAngle, Math.abs(power));//check if someone is pushing you
//                robot.FL.setPower(power - correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
//                robot.FR.setPower(power + correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
//                robot.RR.setPower(power + correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
//                robot.RL.setPower(power - correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)

                robot.setAllGivenPower(power);

                telemetry.addData("Path", "Driving "+" inches");
                telemetry.addData("tickDifference", tickDifference);
                telemetry.addData("power", power);
                telemetry.update();
            }

            robot.stopMotors();

            telemetry.addData("Path", "Complete");
            telemetry.update();

            robot.useEncoders(true);
        }
    }

    public void runToAngle(double targetAngle, double threshold) {
        double angleDifference = targetAngle - robot.getAngle();
        double power = Range.clip(0.005*Math.abs(angleDifference) + 0.2, 0.2, 1);

        if(angleDifference < 0)
            power = -power;

        robot.useEncoders(false);

        robot.FL.setPower(power);
        robot.FR.setPower(-power);
        robot.RL.setPower(power);
        robot.RR.setPower(-power);

        boolean run = true;

        runtime.reset();
        while (opModeIsActive() && run) {
            angleDifference = targetAngle - robot.getAngle();
            power = Range.clip(0.005*Math.abs(angleDifference) + 0.2, 0.2, 1);
            if(angleDifference < 0)
                power = -power;

            robot.FL.setPower(power);
            robot.FR.setPower(-power);
            robot.RL.setPower(power);
            robot.RR.setPower(-power);
            telemetry.addData("angle difference", angleDifference);
            telemetry.addData("power", robot.FL.getPower());
            telemetry.update();
            if(robot.getAngle() == targetAngle)
                run = false;
            if(runtime.seconds() > 2)
                run = false;
        }

        // Step 4:  Stop and close the claw.
        robot.FL.setPower(0);
        robot.FR.setPower(0);
        robot.RL.setPower(0);
        robot.RR.setPower(0);
        sleep(200);

        robot.useEncoders(true);
    }

    public void rotateAngle(double power, double angle, double threshold) {
        angle = -angle;
        if(angle < 0)
            power = -power;

        robot.useEncoders(false);

        robot.FL.setPower(power);
        robot.FR.setPower(-power);
        robot.RL.setPower(power);
        robot.RR.setPower(-power);

        double newAngle = robot.getAngle() + angle;
        boolean run = true;

        runtime.reset();
        while (opModeIsActive() && run) {
            if(Math.abs(robot.getAngle() - newAngle) < threshold)
                run = false;
            if(runtime.seconds() > 4)
                run = false;
        }

        // Step 4:  Stop and close the claw.
        robot.FL.setPower(0);
        robot.FR.setPower(0);
        robot.RL.setPower(0);
        robot.RR.setPower(0);
        sleep(200);

        robot.useEncoders(true);

    }

    public void strafe(double power, double time) {
        robot.FL.setPower(power);
        robot.FR.setPower(-power);
        robot.RL.setPower(-power);
        robot.RR.setPower(power);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 4:  Stop and close the claw.
        robot.FL.setPower(0);
        robot.FR.setPower(0);
        robot.RL.setPower(0);
        robot.RR.setPower(0);
        mySleep(0.2);

    }

    private void strafeGyro(double power, double time) {//uses gyro to make sure robot's angle stays the same - prevents unintentional rotation
        double startAngle = robot.getAngle();
        robot.FLpower = +power;
        robot.FRpower = -power;
        robot.RRpower = +power;
        robot.RLpower = -power;

        robot.setAllPower();

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {//we get on the train together
            double correction = robot.getCorrection(startAngle, Math.abs(power));//check if someone is pushing you
            robot.FL.setPower(+power - correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
            robot.FR.setPower(-power + correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
            robot.RR.setPower(+power + correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
            robot.RL.setPower(-power - correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
        }
        robot.setAllGivenPower(0);//once we're at the station, forget anything happened
        mySleep(0.2);

    }

    public void moveDistanceGyro(double power, double time) {
        double startAngle = robot.getAngle();
        robot.FL.setPower(-power);
        robot.FR.setPower(-power);
        robot.RL.setPower(-power);
        robot.RR.setPower(-power);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {//we get on the train together
            double correction = robot.getCorrection(startAngle, Math.abs(power));//check if someone is pushing you
            robot.FL.setPower(-power - correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
            robot.FR.setPower(-power + correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
            robot.RR.setPower(-power + correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
            robot.RL.setPower(-power - correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        robot.setAllGivenPower(0);
        mySleep(0.2);

    }

    public void mySleep(double time) {//seconds
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {

        }
    }

    public void moveDistance(double power, double time) {

        robot.FL.setPower(power);
        robot.FR.setPower(power);
        robot.RL.setPower(power);
        robot.RR.setPower(power);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 4:  Stop and close the claw.
        robot.FL.setPower(0);
        robot.FR.setPower(0);
        robot.RL.setPower(0);
        robot.RR.setPower(0);
        sleep(200);

    }

    public void moveEncoderDifferential(double distance) {
        double k = 0.0004;

        robot.stopAndResetEncoders();

        robot.useEncoders(true);

        double rotations = distance/ wheelCircumference; //distance / circumference (inches)
        int targetTicks = (int)(rotations*ticksPerRev);

        double tickAvg = (int)((Math.abs(robot.FL.getCurrentPosition()) + Math.abs(robot.FR.getCurrentPosition()) + Math.abs(robot.RL.getCurrentPosition()) + Math.abs(robot.RR.getCurrentPosition()))/4.0);
        double tickDifference = targetTicks - tickAvg;
        double power = Range.clip(k*Math.abs(tickDifference) + 0.2, 0.2, 0.8);

        if(distance < 0)
            power = -power;

        if(opModeIsActive()) {
            robot.RL.setTargetPosition(targetTicks);
            robot.RR.setTargetPosition(targetTicks);
            robot.FL.setTargetPosition(targetTicks);
            robot.FR.setTargetPosition(targetTicks);

            robot.RL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.RR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.setAllGivenPower(power);

            double startAngle = robot.getAngle();

            while(robot.RL.isBusy() || robot.RR.isBusy() || robot.FL.isBusy() || robot.FR.isBusy()) {
                //wait till motor finishes

                tickAvg = (int)((Math.abs(robot.FL.getCurrentPosition()) + Math.abs(robot.FR.getCurrentPosition()) + Math.abs(robot.RL.getCurrentPosition()) + Math.abs(robot.RR.getCurrentPosition()))/4.0);
                tickDifference = targetTicks - tickAvg;
                power = Range.clip(k*Math.abs(tickDifference) + 0.2, 0.2, 0.8);

                if(tickDifference < 0)
                    power = -power;
//                 double correction = robot.getCorrection(startAngle, Math.abs(power));//check if someone is pushing you
//                robot.FL.setPower(power - correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
//                robot.FR.setPower(power + correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
//                robot.RR.setPower(power + correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
//                robot.RL.setPower(power - correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)

                robot.setAllGivenPower(power);

                telemetry.addData("Ticks target", targetTicks);
                telemetry.addData("distance target", distance);

                telemetry.addData("tickDifference", tickDifference);
                telemetry.addData("power", power);
                telemetry.update();
            }

            robot.stopMotors();

            telemetry.addData("Path", "Complete");
            telemetry.update();

            robot.useEncoders(true);
        }
    }

    public void moveDistanceEnc(double power, double distance) {
        robot.stopAndResetEncoders();

        robot.useEncoders(true);

        double rotations = distance/ wheelCircumference; //distance / circumference (inches)
        int targetTicks = (int)(rotations*ticksPerRev);



        if(opModeIsActive()) {
            robot.RL.setTargetPosition(targetTicks);
            robot.RR.setTargetPosition(targetTicks);
            robot.FL.setTargetPosition(targetTicks);
            robot.FR.setTargetPosition(targetTicks);

            robot.RL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.RR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.setAllGivenPower(power);

            double startAngle = robot.getAngle();

            while(robot.RL.isBusy() || robot.RR.isBusy() || robot.FL.isBusy() || robot.FR.isBusy()) {
                //wait till motor finishes working
                double correction = robot.getCorrection(startAngle, Math.abs(power));//check if someone is pushing you
                robot.FL.setPower(power - correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
                robot.FR.setPower(power + correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
                robot.RR.setPower(power + correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
                robot.RL.setPower(power - correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
                telemetry.addData("Path", "Driving "+distance+" inches");
                telemetry.update();
            }

            robot.stopMotors();

            telemetry.addData("Path", "Complete");
            telemetry.update();

            robot.useEncoders(true);
        }
    }


    public void strafeEnc(double power, double distance) {
        robot.stopAndResetEncoders();

        robot.useEncoders(true);

        double rotations = distance/ wheelCircumference; //distance / circumference (inches)
        int targetTicks = (int)(rotations*ticksPerRev);

        if(opModeIsActive()) {
            robot.RL.setTargetPosition(-targetTicks);
            robot.RR.setTargetPosition(targetTicks);
            robot.FL.setTargetPosition(targetTicks);
            robot.FR.setTargetPosition(-targetTicks);

            robot.RL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.RR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.setAllGivenPower(power);

            double startAngle = robot.getAngle();

            while(robot.RL.isBusy() || robot.RR.isBusy() || robot.FL.isBusy() || robot.FR.isBusy()) {
                //wait till motor finishes working
                 double correction = robot.getCorrection(startAngle, Math.abs(power));//check if someone is pushing you
                robot.FL.setPower(power - correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
                robot.FR.setPower(power + correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
                robot.RR.setPower(power + correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
                robot.RL.setPower(power - correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
                telemetry.addData("Path", "Driving "+distance+" inches");
                telemetry.update();
            }

            robot.stopMotors();

            telemetry.addData("Path", "Complete");
            telemetry.update();

            robot.useEncoders(true);
        }
    }

    public void strafeEncoderDifferential(double distance) {
        double k = 0.0004;

        robot.stopAndResetEncoders();

        robot.useEncoders(true);

        double rotations = distance/ wheelCircumference; //distance / circumference (inches)
        int targetTicks = (int)(rotations*ticksPerRev);

        double tickAvg = (int)((robot.FL.getCurrentPosition() + robot.FR.getCurrentPosition() + robot.RL.getCurrentPosition() + robot.RR.getCurrentPosition())/4);
        double tickDifference = targetTicks - tickAvg;
        double power = Range.clip(k*Math.abs(tickDifference) + 0.1, 0.1, 1);

        if(tickDifference < 0)
            power = -power;

        if(opModeIsActive()) {
            robot.RL.setTargetPosition(-targetTicks);
            robot.RR.setTargetPosition(targetTicks);
            robot.FL.setTargetPosition(targetTicks);
            robot.FR.setTargetPosition(-targetTicks);

            robot.RL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.RR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.setAllGivenPower(power);

            double startAngle = robot.getAngle();

            while(robot.RL.isBusy() || robot.RR.isBusy() || robot.FL.isBusy() || robot.FR.isBusy()) {
                //wait till motor finishes

                tickAvg = (int)((Math.abs(robot.FL.getCurrentPosition()) + Math.abs(robot.FR.getCurrentPosition()) + Math.abs(robot.RL.getCurrentPosition()) + Math.abs(robot.RR.getCurrentPosition()))/4.0);
                tickDifference = targetTicks - tickAvg;
                power = Range.clip(k*Math.abs(tickDifference) + 0.1, 0.1, 1);

                if(tickDifference < 0)
                    power = -power;
//                 double correction = robot.getCorrection(startAngle, Math.abs(power));//check if someone is pushing you
//                robot.FL.setPower(power - correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
//                robot.FR.setPower(power + correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
//                robot.RR.setPower(power + correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
//                robot.RL.setPower(power - correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)

                robot.setAllGivenPower(power);

                telemetry.addData("Path", "Driving "+distance+" inches");
                telemetry.addData("tickDifference", tickDifference);
                telemetry.addData("power", power);
                telemetry.update();
            }

            robot.stopMotors();

            telemetry.addData("Path", "Complete");
            telemetry.update();

            robot.useEncoders(true);
        }
    }

    @Override
    public void runOpMode() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.initMotors(hardwareMap);
        robot.initServos(hardwareMap);//servo
        robot.initIMU(hardwareMap);//gyro

        detector.setOffset(1.0f/8f, 1.7f/8f);
        detector.camSetup(hardwareMap);

        robot.useEncoders(true);

//        telemetry.addData("Mode", "calibrating...");
//        telemetry.update();
//        while (!isStopRequested() && !robot.imu.isGyroCalibrated())
//        {
//            sleep(50);
//            idle();
//        }
//
//        telemetry.addData("Mode", "waiting for start");
//        telemetry.addData("imu calib status", robot.imu.getCalibrationStatus().toString());
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        if(opModeIsActive()) {
            detector.updateVals();
            vals = detector.getVals();
            telemetry.addData("Values", vals[1] + "   " + vals[0] + "   " + vals[2]);
            telemetry.addLine("hi");

            telemetry.update();

            if(vals[0] == 0){//middle
                moveEncoderDifferential(8);
                strafeGyro(-0.7, 2);
                robot.leftServoDown();
                mySleep(0.5);
                strafeGyro(0.7, 1);
                moveEncoderDifferential(50);
                robot.leftServoUp();
                mySleep(0.5);
                moveEncoderDifferential(-20);//-73
//                strafeGyro(-0.7, 1);
//                robot.leftServoDown();
//                mySleep(0.5);
//                strafeGyro(1, 0.5);
//                moveEncoderDifferential(90);
//                robot.leftServoUp();
//                mySleep(0.5);
//                moveEncoderDifferential(-30);
            } else if(vals[1] == 0) {//left
                strafeGyro(-0.7, 2);
                robot.leftServoDown();
                mySleep(0.5);
                strafeGyro(0.7, 1);
                moveEncoderDifferential(55);
                robot.leftServoUp();
                mySleep(0.5);
                moveEncoderDifferential(-20);//-73
//                strafeGyro(-0.7, 1);
//                robot.leftServoDown();
//                mySleep(0.5);
//                strafeGyro(1, 0.5);
//                moveEncoderDifferential(90);
//                robot.leftServoUp();
//                mySleep(0.5);
//                moveEncoderDifferential(-30);
            } else {//right
                moveEncoderDifferential(17);
                strafeGyro(-0.7, 2);
                robot.leftServoDown();
                mySleep(0.5);
                strafeGyro(0.7, 1);
                moveEncoderDifferential(45);
                robot.leftServoUp();
                mySleep(0.5);
                moveEncoderDifferential(-20);//-73
//                strafeGyro(-0.7, 1);
//                robot.leftServoDown();
//                mySleep(0.5);
//                strafeGyro(1, 0.5);
//                moveEncoderDifferential(90);
//                robot.leftServoUp();
//                mySleep(0.5);
//                moveEncoderDifferential(-30);
            }


            telemetry.addData("Path", "Complete");
            telemetry.update();
        }
    }
}
