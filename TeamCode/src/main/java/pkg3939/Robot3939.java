package pkg3939;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Math.*;

public class Robot3939 {

    public DcMotor RL, RR, FL, FR;//drive motors
    public DcMotor leftSlides, rightSlides;//linear slide motors
    public Servo servoRight, servoLeft;//autonomous claw servos
    public Servo bar;//foundation mover servo
    public Servo stoneArm;
    public Servo hinge;

    public BNO055IMU imu;//gyro
    private Orientation lastAngles = new Orientation();//saves angles
    double  globalAngle;//robot angle

    public double FLpower, FRpower, RLpower, RRpower;//power of the motors
    public double speed = 10.0;

    public static final double minSpeed = 0.10;
    public static final boolean earthIsFlat = true;

    private final double encoderTicks = 537.6;
    public final double wheelDiameter = 3.85827;//in inches

    public void initMotors(HardwareMap hwmap) {
        RL        = hwmap.dcMotor.get("left_drive");
        RR       = hwmap.dcMotor.get("right_drive");
        FL       = hwmap.dcMotor.get("front_left");
        FR      = hwmap.dcMotor.get("front_right");

        RL.setDirection(DcMotorSimple.Direction.FORWARD);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        RR.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void initLinearSlides(HardwareMap hwmap) {
        leftSlides = hwmap.dcMotor.get("leftSlides");
        rightSlides = hwmap.dcMotor.get("rightSlides");

        leftSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftSlides.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlides.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setFront(HardwareMap hwmap) {
        RL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        RR.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void initServos(HardwareMap hwmap) {
        servoRight = hwmap.servo.get("servoRight");//left foundation
        servoLeft = hwmap.servo.get("servoLeft");//right foundation
        stoneArm = hwmap.servo.get("stoneArm");
        hinge = hwmap.servo.get("hinge");
    }

    public void initIMU(HardwareMap hwmap) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwmap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);
    }

    public void useEncoders(boolean status) {
        if(status) {
            RL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            RL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void stopAndResetEncoders() {
        RL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    boolean LTheld = false;
    boolean RTheld = false;
    public void setSpeed(boolean LTpressed, boolean RTpressed) {
        if(!LTheld && LTpressed) {
            LTheld = true;
            speed--;
        } else if(!LTpressed) {
            LTheld = false;
        }

        if(!RTheld && RTpressed) {
            RTheld = true;
            speed++;
        } else if(!RTpressed) {
            RTheld = false;
        }

        speed = Range.clip(speed, 0, 10);
    }

    public void setAllGivenPower(double power) {
        FL.setPower(power);
        RL.setPower(power);
        FR.setPower(power);
        RR.setPower(power);
    }

    public void setAllPower() {
        FL.setPower(FLpower);
        RL.setPower(RLpower);
        FR.setPower(FRpower);
        RR.setPower(RRpower);
    }

    public void stopMotors() {
        setAllGivenPower(0);
    }

    public static double[] getComponents(double x, double y, double offset) {//offset = imu
        double stickAngle = returnAngle(x, y);//if stick up, stickangle = 90
        double offsetAngle = -stickAngle + offset;//stick - IMU

        if(offsetAngle < 0)//get rid of negative angle
            offsetAngle += 360.0;
        double[] offsetPoint = new double[2];
        offsetPoint[0] = getHypotenuse(x, y)*Math.cos(Math.toRadians(offsetAngle));
        offsetPoint[1] = getHypotenuse(x, y)*Math.sin(Math.toRadians(offsetAngle));
        return offsetPoint;
    }

    //Returns the angle in degrees from the origin to the specified point
    public static double returnAngle(double x, double y) {
        double[] angleDirection = new double[2];
        angleDirection[0] = x - 0;
        angleDirection[1] = y - 0;
        return Math.toDegrees(Math.atan2(angleDirection[1], angleDirection[0]));
    }

    public static double getHypotenuse(double x, double y) {
        return Math.sqrt(x*x + y*y);
    }

    public void drive(double LX, double LY, double rotate) {
        if((abs(LX) > minSpeed) || (abs(LY) > minSpeed) || (abs(rotate) > minSpeed)) {
            FLpower = LY - LX + rotate;
            FRpower = LY + LX - rotate;
            RRpower = LY - LX - rotate;
            RLpower = LY + LX + rotate;
        } else if (earthIsFlat) { //stop robot
            FLpower = 0;
            FRpower = 0;
            RRpower = 0;
            RLpower = 0;
        }

        //get max power out of all 4 powers
        double maxPower = max(1.0, max(max(abs(FLpower), abs(RLpower)), max(abs(FRpower), abs(RRpower))));

        //if any of them is greater than 1, it will slow down all by the same ratio, for smoother control
        if(maxPower > 1.0) {
            FLpower /= maxPower;
            FRpower /= maxPower;
            RLpower /= maxPower;
            RRpower /= maxPower;
        }
        double reduction = 312.0/435.0;//front wheels are 435 rpm, back wheels are 312 rpm
        // so we have to decrease their speed to match that of the rear wheels.
        FL.setPower(FLpower*speed/10.0);
        FR.setPower(FRpower*speed/10.0);
        RL.setPower(RLpower*speed/10.0);
        RR.setPower(RRpower*speed/10.0);
    }

    //for autonomous, use only one axis at a time for now. still under development
    public void driveToPosition(double x, double y, double rotationAngle, double power) {
        RL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        useEncoders(true);

        //y
        double distancePerRotationY = 3.1415 * wheelDiameter; //pi * diameter (inches) = circumference
        double rotationsY = y/distancePerRotationY; //distance / circumference (inches)
        int encoderTargetY = (int)(rotationsY*encoderTicks);

        //x
        double distancePerRotationX = 13.5; //distance per rotations is different than circumference when strafing (inches)
        double rotationsX = x/distancePerRotationX; //distance / circumference (inches)
        int encoderTargetX = (int)(rotationsX*encoderTicks);

        //rotationAngle
        double ticksPerRotation = 0;//measure how many ticks for a 360 rotationAngle
        double rotationsA = rotationAngle/360;
        int encoderTargetA = (int)(rotationsA*ticksPerRotation);

        RL.setTargetPosition(encoderTargetY-encoderTargetX+encoderTargetA);
        RR.setTargetPosition(encoderTargetY+encoderTargetX-encoderTargetA);
        FL.setTargetPosition(encoderTargetY+encoderTargetX+encoderTargetA);
        FR.setTargetPosition(encoderTargetY-encoderTargetX-encoderTargetA);

        RL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setAllGivenPower(abs(power));

        stopMotors();

        useEncoders(true);
    }

    boolean barUp = true;
    boolean aHeld = false;
    boolean a2Held = false;
    boolean b2Held = false;
    boolean hingeTurn = false;
    boolean stoneArmGrab = false;

    public boolean slidesDown() {
        if(leftSlides.getPower() == 0 && rightSlides.getPower() == 0)
            return true;
        return false;
    }

    public void leftServoDown() {
        servoLeft.setPosition(0.33);
    }

    public void leftServoUp() {
        servoLeft.setPosition(0.66);
    }

    public void rightServoDown() {
        servoRight.setPosition(0.32);
    }

    public void rightServoUp() {
        servoRight.setPosition(0);
    }

    public void foundationDown() {
        leftServoDown();
        rightServoDown();
    }

    public void foundationUp() {
        rightServoUp();
        leftServoUp();
    }

    public void setHinge(boolean b2Pressed) {
        if(!b2Held && b2Pressed) {
            b2Held = true;
            hingeTurn = !hingeTurn;
        } else if(!b2Pressed)
            b2Held = false;

        if(hingeTurn)//down
            hinge.setPosition(0.735);
        else//up
            hinge.setPosition(0.065 );
    }

    public void setStoneArm(boolean a2Pressed) {
        if(!a2Held && a2Pressed) {
            a2Held = true;
            stoneArmGrab = !stoneArmGrab;
        } else if(!a2Pressed)
            a2Held = false;

        if(stoneArmGrab)//down
            stoneArm.setPosition(0.03);
        else//up
            stoneArm.setPosition(0.33);
    }

    public void hookFoundation(boolean aPressed) {
        if (!aHeld && aPressed) {
            aHeld = true;
            barUp = !barUp;
        } else if (!aPressed)
            aHeld = false;

        if (barUp) {//up
            foundationUp();
        } else if(earthIsFlat) {//down
            foundationDown();
        }
    }

    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    public double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */

    public double getCorrection(double startAngle, double power)
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .125 * power;

        angle = getAngle();

//        if (angle == 0)
//            correction = 0;             // no adjustment.
//        else
//            correction = -angle;        // reverse sign of angle for correction.

        correction = angle - startAngle;

        correction *= gain;

        correction = Range.clip(correction, -1, 1);

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power)
    {
        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            FLpower = power;
            RLpower = power;
            FRpower = -power;
            RRpower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            FLpower = -power;
            RLpower = -power;
            FRpower = power;
            RRpower = power;
        }
        else
            return;

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (getAngle() == 0) {}

            while (getAngle() > degrees) {}
        }
        else    // left turn.
            while (getAngle() < degrees) {}

        // turn the motors off.
        stopMotors();

        // wait for rotation to stop.
        while(RL.isBusy() || RR.isBusy() || FR.isBusy() || FL.isBusy()) {}

        // reset angle tracking on new heading.
        resetAngle();
    }
}
