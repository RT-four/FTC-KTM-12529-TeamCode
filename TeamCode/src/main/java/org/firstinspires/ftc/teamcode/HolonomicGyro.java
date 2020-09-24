package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import pkg3939.Robot3939;

/**
 * Created by maryjane on 11/6/2018.
 * after 1st meet
 * mecanum wheels
 * Hi, edit
 */

@TeleOp(name="HolonomicGyro", group="Iterative Opmode")
//@Disabled
public class HolonomicGyro extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    //Declaration of the motors and servos goes here
    Robot3939 robot = new Robot3939();

    public static final boolean earthIsFlat = true;

    public void moveSlides(double power, int constant) {
        if (opModeIsActive()) {
            robot.leftSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.leftSlides.setTargetPosition(robot.leftSlides.getCurrentPosition() + constant);
            robot.rightSlides.setTargetPosition(robot.rightSlides.getCurrentPosition() + constant);

            robot.leftSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.leftSlides.setPower(power);
            robot.rightSlides.setPower(power);

            runtime.reset();

            while (robot.leftSlides.isBusy() || robot.rightSlides.isBusy()) {
                //wait till motor finishes working
                robot.drive(gamepad1.left_stick_x,
                        gamepad1.left_stick_y,
                        gamepad1.right_stick_x);
                telemetry.addLine("Slides Extending");
                telemetry.update();
                if (runtime.seconds() > 1.2)
                    break;
            }
            telemetry.addLine("Extended");
            telemetry.update();

            robot.leftSlides.setPower(0);
            robot.rightSlides.setPower(0);

            robot.leftSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            robot.leftSlides.setPower(-0.25);
            robot.rightSlides.setPower(-0.25);
            //
//            robot.leftSlides.setPower(0);
//            robot.rightSlides.setPower(0);
        }
    }

    @Override //when init is pressed
    public void runOpMode() {
        robot.initMotors(hardwareMap);
        robot.initServos(hardwareMap);
        robot.setFront(hardwareMap);
        //robot.initIMU(hardwareMap);
        robot.useEncoders(false);//don't need encoders for teleop
        robot.initLinearSlides(hardwareMap);

        boolean driver = true;
        boolean yHeld = false;
        boolean lsbHeld = false;
        boolean useNormal = true;
        boolean dUpHeld = false;
        boolean initIMU = true;

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            //forks
            double LX = gamepad1.left_stick_x, LY = -gamepad1.left_stick_y, RX = -gamepad1.right_stick_y;

            robot.hookFoundation(gamepad1.a);//pressing a changes claw position, up to down, or vice versa
            robot.setSpeed(gamepad1.left_bumper, gamepad1.right_bumper);
            robot.setHinge(gamepad2.b);
            robot.setStoneArm(gamepad2.a);

            if (gamepad2.right_bumper)
                moveSlides(0.5, 20);
            else if (gamepad2.left_bumper)
                moveSlides(1, -50);
            else if (gamepad2.y && robot.slidesDown())
                moveSlides(1, -225);
            else if (gamepad2.x) {
                robot.leftSlides.setPower(0);
                robot.rightSlides.setPower(0);
            }

            //driver 1 control switch
            if (!yHeld && gamepad1.y) {
                yHeld = true;
                driver = !driver;
            } else if (!gamepad1.y) {
                yHeld = false;
            }

            //driver 2 control switch
            if(!dUpHeld && gamepad2.dpad_up) {
                dUpHeld = true;
                driver = !driver;
            } else if(gamepad2.dpad_up)
                dUpHeld = false;

            //press to switch between Gyro and Normal Drive
            if(!lsbHeld && gamepad1.left_stick_button) {
                lsbHeld = true;
                useNormal = !useNormal;
            } else if(!gamepad1.left_stick_button)
                lsbHeld = false;

            if(!useNormal) {
                if(initIMU) {
                    robot.initIMU(hardwareMap);
                    initIMU = false;
                }
                double angle = robot.getAngle();
                if(angle < 0)
                    angle += 360.0;

                //robot drive
                double coords[] = robot.getComponents(LX, LY, angle);
                double newLX = coords[0];
                double newLY = coords[1];

                robot.drive(newLX,newLY,RX);
            } else {
                robot.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            }

            telemetry.addData("Drive", "Holonomic");
            //telemetry.addData("Global Heading", robot.getAngle());
            telemetry.addData("LX", gamepad1.left_stick_x);
            telemetry.addData("LY", gamepad1.left_stick_y);
            telemetry.addData("RX", gamepad1.right_stick_y);

            telemetry.addData("speed", robot.speed);
            telemetry.addData("left servo", robot.servoLeft.getPosition());
            telemetry.addData("right servo", robot.servoRight.getPosition());
            telemetry.addData("hinge", robot.hinge.getPosition());
            telemetry.addData("stoneArm", robot.stoneArm.getPosition());

            telemetry.update();
        }
    }
}


/*
double angle = robot.getAngle();
            if(angle < 0)
                angle += 360.0;

            //robot drive
            double coords[] = robot.getComponents(LX, LY, angle);
            double newLX = coords[0];
            double newLY = coords[1];

            robot.drive(newLX,newLY,RX);
 */
