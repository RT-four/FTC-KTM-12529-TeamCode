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


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a four wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "KTM TeleOp 2019 (ALT)", group = "Linear Opmode")

//@Disabled
public class TeleOP2019 extends LinearOpMode {
    private static final int LED_CHANNEL = 5;
    //    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //Cha
    private DcMotor m6Intake = null;
    private DcMotor m5Lift = null;
    private Servo s5Shovel = null;

    //-------
    double magic(double input) {
        if(Math.abs(input)<0.02){
            double nol=0;
            return nol;
        } else{
            return Math.signum(input) * (0.9*Math.pow(Math.abs(input), 2)+0.1);
        }
    }
    //переменные для ПИД
    //int setpoint = 0;   // заданная величина, которую должен поддерживать регулятор
    //double input = 0;      // сигнал с датчика (например температура, которую мы регулируем)
    //double output = 0;     // выход с регулятора на управляющее устройство (например величина ШИМ или угол поворота серво)
    double pidMin = 0;     // минимальный выход с регулятора
    double pidMax = 255;   // максимальный выход с регулятора
    // коэффициенты
    double Kp = 0.0;
    double Ki = 0.0;
    double Kd = 1.2;
    double _dt_s = 0.1; // время итерации в секундах//скорее всего получится выцепить из таймера
    // вспомогательные переменные
    double prevInput = 0;
    double integral = 0.0;

    //------------------
    double computePID(double input,double prevInput, double setpoint, double time, double time1) {
        double error = setpoint - input;           // ошибка регулирования
        double delta_input = prevInput - input;    // изменение входного сигнала
        prevInput = input;
        double output = 0;
        output += error * Kp;                  // пропорционально ошибке регулирования
        double dt=time1-time;
        output += delta_input * Kd / _dt_s;    // дифференциальная составляющая
        integral += error * Ki * _dt_s;        // расчёт интегральной составляющей
        // тут можно ограничить интегральную составляющую!
        output += integral;                           // прибавляем интегральную составляющую
        //output = constrain(output, pidMin, pidMax);   // ограничиваем выход
        return output;
    }

    /*
     * Functions declaration
     */
    //Lift claw
    void liftClaw(double lift_power) {
        m5Lift.setPower(lift_power);
    }

    void shovelTrigger(double shovel_pos) {
        s5Shovel.setPosition(shovel_pos);
    }
    
    void setPowerTimed(DcMotor motor, double power, long milliseconds) {
        motor.setPower(power);
        sleep(milliseconds);
        motor.setPower(0);

    }

    void setPowerTimed(CRServo Crservo, double power, long milliseconds) {
        Crservo.setPower(power);
        sleep(milliseconds);
        Crservo.setPower(0);

    }

    /*
     *Relic related
     */
    // Grab relic
    // Extend grabbing component
    // Retract grabbing component
    /**
     * End of functions declaration
     */
    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        // Chassis
        DcMotor m1Drive = hardwareMap.get(DcMotor.class, "m1 drive");
        DcMotor m2Drive = hardwareMap.get(DcMotor.class, "m2 drive");
        DcMotor m3Drive = hardwareMap.get(DcMotor.class, "m3 drive");
        DcMotor m4Drive = hardwareMap.get(DcMotor.class, "m4 drive");
        DcMotor m5Lift = hardwareMap.get(DcMotor.class, "m5 lift");
        DcMotor m6Intake = hardwareMap.get(DcMotor.class, "m6 intake");
        DcMotor m7ruletka = hardwareMap.get(DcMotor.class, "m7 rul");
        CRServo s1RelicExtRet = hardwareMap.get(CRServo.class, "s1 top claw");
        //s2_bottom_Claw = hardwareMap.get(CRServo.class, "s2 bottom claw");
        Servo s3Rotation = hardwareMap.get(Servo.class, "s3 rotation");
        Servo s4Kicker = hardwareMap.get(Servo.class, "s4 kick");
        Servo s5Shovel = hardwareMap.get(Servo.class, "s5 shovel");
        Servo s6RelicClaw = hardwareMap.get(Servo.class, "s6 relic claw");
        Servo s7RelicArm = hardwareMap.get(Servo.class, "s7 relic arm");
        //DistanceSensor DistanceSensor_left = hardwareMap.get(DistanceSensor.class,"dist left");
        //DistanceSensor DistanceSensor_right = hardwareMap.get(DistanceSensor.class,"dist right");
        //DistanceSensor DistanceSensor_back = hardwareMap.get(DistanceSensor.class,"dist back");
        //DistanceSensor DistanceSensor_forward = hardwareMap.get(DistanceSensor.class,"dist forward");
        //BNO055IMU imu;
        //Orientation angles;
        //BNO055IMU.Parameters parametrs = new BNO055IMU.Parameters();
        //parametrs.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        //imu = hardwareMap.get(BNO055IMU.class, "imu rev");
        //imu.initialize(parametrs);
        //sensor
        //TouchSensor touchSensor = hardwareMap.get(TouchSensor.class, "sensor touch");

        //-------
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        m1Drive.setDirection(DcMotor.Direction.FORWARD);
        m2Drive.setDirection(DcMotor.Direction.FORWARD);
        m3Drive.setDirection(DcMotor.Direction.FORWARD);
        m4Drive.setDirection(DcMotor.Direction.FORWARD);
        m5Lift.setDirection(DcMotor.Direction.FORWARD);
        m6Intake.setDirection(DcMotor.Direction.FORWARD);
        //DcMotor m6Intake=hardwareMap.dcMotor.get("m6Intake");
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
        double m5LiftPower;
        double m6IntakePower;
        double m1DrivePowerforrotation=0;
        double m2DrivePowerforrotation=0;
        double m3DrivePowerforrotation=0;
        double m4DrivePowerforrotation=0;
        double m1DrivePowerfordrivetofoundation=0;
        double m2DrivePowerfordrivetofoundation=0;
        double m3DrivePowerfordrivetofoundation=0;
        double m4DrivePowerfordrivetofoundation=0;
        double m1DrivePowerfordrivetofoundation1=0;
        double m2DrivePowerfordrivetofoundation1=0;
        double m3DrivePowerfordrivetofoundation1=0;
        double m4DrivePowerfordrivetofoundation1=0;
        double m1DrivePowerfordrivetofoundation11=0;
        double m2DrivePowerfordrivetofoundation11=0;
        double m3DrivePowerfordrivetofoundation11=0;
        double m4DrivePowerfordrivetofoundation11=0;
        double m1DrivePowerfordrivetofoundation2=0;
        double m2DrivePowerfordrivetofoundation2=0;
        double m3DrivePowerfordrivetofoundation2=0;
        double m4DrivePowerfordrivetofoundation2=0;
        double m1DrivePowerfordrivetofoundation22=0;
        double m2DrivePowerfordrivetofoundation22=0;
        double m3DrivePowerfordrivetofoundation22=0;
        double m4DrivePowerfordrivetofoundation22=0;
        double a=0;
        double b=0;
        double prevangel=0;

        while (opModeIsActive()) {

//int ANDYMARK_TICKS_PER_REV = 1120;
            /*
             * Chassis movement
             */
            //Setup a variable for each drive wheel to save power level for telemetry


            // POV Mode uses right stick to go forward and right to slide.
            // - This uses basic math to combine motions and is easier to drive straight.
            double driveL = -gamepad1.left_stick_y;
            double driveR = -gamepad1.right_stick_y;
            float relic = gamepad2.left_stick_x;
            boolean servo31 = gamepad1.y;
            boolean servo32 = gamepad1.x;
           // boolean servomarker = ;
            boolean ruletka_tuda = gamepad2.dpad_up;
            boolean ruletka_suda = gamepad2.dpad_down;
            boolean initpodiezd = gamepad1.b;
            boolean podiezd = gamepad1.x;
            boolean vverh = gamepad2.x;
            boolean zagreb = gamepad2.a;
            double podiem = gamepad2.left_stick_y;
            boolean servocupstone= gamepad2.dpad_down;
            boolean servo_tyapka_verh = gamepad2.y;
            boolean servo_tyapka_niz = gamepad2.a;
            double servo_tyapka=gamepad2.right_trigger;
           // double servor = gamepad2.right_trigger;
            double servor = gamepad2.right_trigger;
            double slideL = 0.7*gamepad1.left_trigger;
            double slideR = 0.7*-gamepad1.right_trigger;
            double vpernazad=gamepad1.left_stick_y;
            double vleovpravo= -gamepad1.left_stick_x;
            double povorot= 0.7*gamepad1.right_stick_x;
            boolean zahvatcube=gamepad2.y;
            //DeviceInterfaceModule cdim = hardwareMap.deviceInterfaceModule.get("dim");
            //Slide Related
            slideL=magic(slideL);
            slideR=magic(slideR);
            povorot=magic(povorot);
            vpernazad=magic(vpernazad);
            //vleovpravo=magic(vleovpravo);
                /*m2DrivePower = povorot-vpernazad-vleovpravo;
                m4DrivePower = povorot+vpernazad-vleovpravo;
                m1DrivePower = povorot+vpernazad+vleovpravo;
                m3DrivePower = povorot-vpernazad+vleovpravo;*/
            m2DrivePower = (m1DrivePowerfordrivetofoundation2+m1DrivePowerfordrivetofoundation22+m1DrivePowerfordrivetofoundation1+m1DrivePowerfordrivetofoundation11) + povorot-vpernazad-(slideL+slideR+vleovpravo);
            m4DrivePower = (m2DrivePowerfordrivetofoundation2+m2DrivePowerfordrivetofoundation22+m2DrivePowerfordrivetofoundation1+m2DrivePowerfordrivetofoundation11)+ povorot+vpernazad-(slideL+slideR+vleovpravo);
            m1DrivePower = (m3DrivePowerfordrivetofoundation2+m3DrivePowerfordrivetofoundation22+m3DrivePowerfordrivetofoundation1+m3DrivePowerfordrivetofoundation11)+povorot+vpernazad+(slideL+slideR+vleovpravo);
            m3DrivePower = (m4DrivePowerfordrivetofoundation2+m4DrivePowerfordrivetofoundation22+m4DrivePowerfordrivetofoundation1+m4DrivePowerfordrivetofoundation11)+ povorot-vpernazad+(slideL+slideR+vleovpravo);
            double mochs=1;
            double max = Math.max(Math.max(m1DrivePower, m2DrivePower), Math.max(m3DrivePower, m4DrivePower));
            // Send calculated power to wheelsв
            if (max >= 1) {
                m1Drive.setPower(mochs*m1DrivePower *1/ max);
                m2Drive.setPower(mochs*m2DrivePower *1/ max);
                m3Drive.setPower(mochs*m3DrivePower *1/ max);
                m4Drive.setPower(mochs*m4DrivePower *1/ max);
            } else {
                m1Drive.setPower(mochs*m1DrivePower*1);
                m2Drive.setPower(mochs*m2DrivePower*1);
                m3Drive.setPower(mochs*m3DrivePower*1);
                m4Drive.setPower(mochs*m4DrivePower*1);
            }


            /*
             * End of chassis related code.
             */
            double servo5;
            //серво пишка
            if(gamepad2.b){ servor=1.42;}
            //подъёмник
            if(podiem!=0){
                if(podiem>0) {
                    m5Lift.setPower(podiem);
                }else{
                    m5Lift.setPower(podiem);
                }
            }else{m5Lift.setPower(0);}

            if(podiem!=0){
                //m5Lift.setPower(podiem*0.65);
                if(podiem>0){
                    m6Intake.setPower(-podiem);
                }else{
                    m6Intake.setPower(-podiem);}
            }else{m6Intake.setPower(0);}
            //захват кубика
            if (servor!=0) { if(gamepad2.b){s3Rotation.setPosition(servor);}else{s3Rotation.setPosition(0.7*servor);} }else{s3Rotation.setPosition(0.7*servor);}
            // if(zahvatcube){s3Rotation.setPosition(1);}else{s3Rotation.setPosition(0);}
            //тяпка:
            if (servo_tyapka_verh) { s4Kicker.setPosition(1); }
            if (servo_tyapka_niz) { s4Kicker.setPosition(0); }
            // if(servo_tyapka>=0){s4Kicker.setPosition(servo_tyapka);}

            //крюки на захват фундамента:
            if (servo32) { servo5=0; if(servo5==0){ s5Shovel.setPosition(0);}}
            if (servo31) {servo5=2;if(servo5==2){ s5Shovel.setPosition(0.45);}}
            //ruletka
            if(ruletka_suda){
                m7ruletka.setPower(1);
            }else{
                m7ruletka.setPower(0);
            }
            if(ruletka_tuda){
                m7ruletka.setPower(-1);
            }else{ m7ruletka.setPower(0);}

            //if(initpodiezd){
            //    imu.initialize(parametrs);
            //}
            //angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            /*if(podiezd==true) {
                a++;
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                if(((angles.firstAngle)>1||(angles.firstAngle)<-1)){
                    if(((angles.firstAngle)>-10&&(angles.firstAngle)<-1)||((angles.firstAngle)<10&&(angles.firstAngle)>1)) {
                        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                        m1DrivePowerforrotation = ((angles.firstAngle) / 60);
                        m2DrivePowerforrotation = ((angles.firstAngle) / 60);
                        m3DrivePowerforrotation = ((angles.firstAngle) / 60);
                        m4DrivePowerforrotation = ((angles.firstAngle) / 60);
                        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    }else if (((angles.firstAngle)>1||(angles.firstAngle)<-1)){
                        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                        m1DrivePowerforrotation = ((angles.firstAngle) / 60);
                        m2DrivePowerforrotation = ((angles.firstAngle) / 60);
                        m3DrivePowerforrotation = ((angles.firstAngle) / 60);
                        m4DrivePowerforrotation = ((angles.firstAngle) / 60);
                        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    }
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                }else {
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    m1DrivePowerforrotation=0;
                    m2DrivePowerforrotation=0;
                    m3DrivePowerforrotation=0;
                    m4DrivePowerforrotation=0;
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                }
                //--------------------------------------------------------------
                if(((angles.firstAngle)<20&&(angles.firstAngle)>-20)){
                    double r=30;
                   if(DistanceSensor_left.getDistance(DistanceUnit.CM)>r+2&&b++>-10){
                       angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                       m1DrivePowerfordrivetofoundation=(r-DistanceSensor_left.getDistance(DistanceUnit.CM))/100;
                       m2DrivePowerfordrivetofoundation=(r-DistanceSensor_left.getDistance(DistanceUnit.CM))/100;
                       m3DrivePowerfordrivetofoundation=-1*(r-DistanceSensor_left.getDistance(DistanceUnit.CM))/100;
                       m4DrivePowerfordrivetofoundation=-1*(r-DistanceSensor_left.getDistance(DistanceUnit.CM))/100;
                       angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                   }else {
                       if (DistanceSensor_left.getDistance(DistanceUnit.CM) < r - 2 && b++ > -10) {
                           angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                           m1DrivePowerfordrivetofoundation = (r - DistanceSensor_left.getDistance(DistanceUnit.CM)) / 100;
                           m2DrivePowerfordrivetofoundation = (r - DistanceSensor_left.getDistance(DistanceUnit.CM)) / 100;
                           m3DrivePowerfordrivetofoundation = -1 * (r - DistanceSensor_left.getDistance(DistanceUnit.CM)) / 100;
                           m4DrivePowerfordrivetofoundation = -1 * (r - DistanceSensor_left.getDistance(DistanceUnit.CM)) / 100;
                           angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                       } else {
                           angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                           m1DrivePowerfordrivetofoundation = 0;
                           m2DrivePowerfordrivetofoundation = 0;
                           m3DrivePowerfordrivetofoundation = 0;
                           m4DrivePowerfordrivetofoundation = 0;
                           angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                       }
                   }
                }else{
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    m1DrivePowerfordrivetofoundation=0;
                    m2DrivePowerfordrivetofoundation=0;
                    m3DrivePowerfordrivetofoundation=0;
                    m4DrivePowerfordrivetofoundation=0;
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                }


            }else{
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                m1DrivePowerforrotation=0;
                m2DrivePowerforrotation=0;
                m3DrivePowerforrotation=0;
                m4DrivePowerforrotation=0;
                m1DrivePowerfordrivetofoundation=0;
                m2DrivePowerfordrivetofoundation=0;
                m3DrivePowerfordrivetofoundation=0;
                m4DrivePowerfordrivetofoundation=0;
                m1DrivePowerfordrivetofoundation1=0;
                m2DrivePowerfordrivetofoundation1=0;
                m3DrivePowerfordrivetofoundation1=0;
                m4DrivePowerfordrivetofoundation1=0;
            }*/
            if(podiezd==true) {

                /*a++;
                prevangel=angles.firstAngle;
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                m1DrivePowerforrotation= computePID(angles.firstAngle,prevangel,0,0.1, 0.2);
                m2DrivePowerforrotation= computePID(angles.firstAngle,prevangel, 0,0.1, 0.2);
                m3DrivePowerforrotation= computePID(angles.firstAngle,prevangel, 0,0.1, 0.2);
                m4DrivePowerforrotation= computePID(angles.firstAngle,prevangel, 0,0.1, 0.2);*/

                //m1DrivePowerforrotation = ((angles.firstAngle) / 60);
                //m2DrivePowerforrotation = ((angles.firstAngle) / 60);
                //m3DrivePowerforrotation = ((angles.firstAngle) / 60);
                //m4DrivePowerforrotation = ((angles.firstAngle) / 60);
            }
            if(gamepad1.dpad_up){
                m1DrivePowerfordrivetofoundation11=0.2;
                m2DrivePowerfordrivetofoundation11=-0.2;
                m3DrivePowerfordrivetofoundation11=-0.2;
                m4DrivePowerfordrivetofoundation11=0.2;
            }else{
                m1DrivePowerfordrivetofoundation11=0;
                m2DrivePowerfordrivetofoundation11=0;
                m3DrivePowerfordrivetofoundation11=0;
                m4DrivePowerfordrivetofoundation11=0;
            }
            if(gamepad1.dpad_down){
                m1DrivePowerfordrivetofoundation1=-0.2;
                m2DrivePowerfordrivetofoundation1=0.2;
                m3DrivePowerfordrivetofoundation1=0.2;
                m4DrivePowerfordrivetofoundation1=-0.2;
            }else{
                m1DrivePowerfordrivetofoundation1=0;
                m2DrivePowerfordrivetofoundation1=0;
                m3DrivePowerfordrivetofoundation1=0;
                m4DrivePowerfordrivetofoundation1=0;
            }

            if(gamepad1.dpad_left){
                m1DrivePowerfordrivetofoundation2=-0.33;
                m2DrivePowerfordrivetofoundation2=-0.33;
                m3DrivePowerfordrivetofoundation2=0.33;
                m4DrivePowerfordrivetofoundation2=0.33;
            }else{
                m1DrivePowerfordrivetofoundation2=0;
                m2DrivePowerfordrivetofoundation2=0;
                m3DrivePowerfordrivetofoundation2=0;
                m4DrivePowerfordrivetofoundation2=0;
            }
            if(gamepad1.dpad_right){
                m1DrivePowerfordrivetofoundation22=0.33;
                m2DrivePowerfordrivetofoundation22=0.33;
                m3DrivePowerfordrivetofoundation22=-0.33;
                m4DrivePowerfordrivetofoundation22=-0.33;
            }else{
                m1DrivePowerfordrivetofoundation22=0;
                m2DrivePowerfordrivetofoundation22=0;
                m3DrivePowerfordrivetofoundation22=0;
                m4DrivePowerfordrivetofoundation22=0;
            }
            //s5Shovel.setPosition(0);
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("a: ", a);
            telemetry.addData("b: ", b);
            //telemetry.addData("angleofrotate", angles.firstAngle);
            telemetry.addData("previnput", prevangel);
            //telemetry.addData("Distance left: ", DistanceSensor_left.getDistance(DistanceUnit.CM));
            telemetry.addData("Motors", "m1Drive (%.2f), m2Drive (%.1f), m3Drive (%.2f), m4Drive (%.2f)", m1DrivePower, m2DrivePower, m3DrivePower, m4DrivePower);
            telemetry.addData("Motors power for rotation", "m1Drive (%.2f), m2Drive (%.1f), m3Drive (%.2f), m4Drive (%.2f)", m1DrivePowerforrotation, m2DrivePowerforrotation, m3DrivePowerforrotation, m4DrivePowerforrotation);
            telemetry.update();
            //cdim.setDigitalChannelState(LED_CHANNEL, false);
        }
    }
}
