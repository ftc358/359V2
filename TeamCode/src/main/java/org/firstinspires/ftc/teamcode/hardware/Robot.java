package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;


public class Robot {

    //Hardware Declaration

    ////Electronics:
    public static DcMotorEx leftFront;
    public static DcMotorEx leftBack;
    public static DcMotorEx rightFront;
    public static DcMotorEx rightBack; //Drivetrain Motors
    public DcMotorEx lift;
    public DcMotorEx horExt;
    public DcMotorEx intake;
    public static DcMotorEx climb; //Accessory Motors
    public static Servo diffy1;
    public static Servo diffy2;
    public static Servo wrist1;
    public static Servo wrist2;
    public static Servo claw1;
    public static Servo claw2;
    public static Servo intakeWrist;
    public static Servo intakePivot1;
    public static Servo intakePivot2;
    public Servo plane; //Servo Motors
    public RevBlinkinLedDriver led;

    ////Sensors:
    public BHI260IMU imu;//IMU on Control Hub
    public AnalogInput diffy1Enc, diffy2Enc, intakePivot1Enc, intakePivot2Enc; //Axon Encoder Readouts
    public ColorRangeSensor In1Color, In2Color; //Color Sensors for Intake
    public DistanceSensor frontDistance;
    public DistanceSensor backDistance; //Possible Front or Back Distance Sensors
    public TouchSensor outtakeTouch, liftReset, intakeReset, horReset;

    ////Misc:
    public VoltageSensor voltageCh, voltageExh; //Voltage Sensor for Control Hub and Expansion Hub
    public LynxModule ch, exh; //The Control and Expansion Hub



    // Define a constructor that allows the OpMode to pass a reference to itself.
    public Robot(HardwareMap hardwareMap) {
        ch = (LynxModule) hardwareMap.get(LynxModule.class, "Control Hub");//Gets
        exh = (LynxModule) hardwareMap.get(LynxModule.class, "Expansion Hub 5");
        ch.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO); //Sets BulkCache for Faster Write on Both
        exh.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        //Fetch and Configure Drive Motors
        /*
        Control Hub:
        M0: leftFront
        M1  leftBack
        M2  rightBack
        M3  rightFront

        S0: diffy1
        S1: diffy2
        S2:
        S3: intakeWrist
        S4: intakePivot1
        S5: intakePivot2

        D0: outtakeTouch
        I2CB0: imu BHI
        I2CB2: backDistance


         */

        leftFront = hardwareMap.get(DcMotorEx.class,"leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class,"leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class,"rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class,"rightBack");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Fetch Accessory Motors
        lift = hardwareMap.get(DcMotorEx.class,"lift");
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        horExt = hardwareMap.get(DcMotorEx.class,"horExt");
        intake = hardwareMap.get(DcMotorEx.class,"intake");
        climb = hardwareMap.get(DcMotorEx.class,"climb");

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horExt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        climb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //Fetch Servos in the Future Here when Installed;
        diffy1 = hardwareMap.get(Servo.class,"diffy1");
        diffy2 = hardwareMap.get(Servo.class,"diffy2");
        diffy1.setDirection(Servo.Direction.REVERSE);

        wrist1 = hardwareMap.get(Servo.class,"wrist1");
        wrist2 = hardwareMap.get(Servo.class,"wrist2");
        wrist2.setDirection(Servo.Direction.REVERSE);
        claw1 = hardwareMap.get(Servo.class,"claw1");
        claw2 = hardwareMap.get(Servo.class,"claw2");
        claw2.setDirection(Servo.Direction.REVERSE);

        intakeWrist = hardwareMap.get(Servo.class,"intakeWrist");
        intakePivot1 = hardwareMap.get(Servo.class,"intakePivot1");
        intakePivot2 = hardwareMap.get(Servo.class,"intakePivot2");
        intakePivot1.setDirection(Servo.Direction.REVERSE);






        //Fetch Blinkin
        led = hardwareMap.get(RevBlinkinLedDriver.class,"led");

        //Fetch Sensors
        imu = hardwareMap.get(BHI260IMU.class,"imu");
        In1Color = hardwareMap.get(RevColorSensorV3.class,"In1Color");
        In2Color = hardwareMap.get(RevColorSensorV3.class,"In2Color");
        backDistance = hardwareMap.get(DistanceSensor.class,"backDistance");
        outtakeTouch = hardwareMap.get(TouchSensor.class,"outtakeTouch");
        horReset = hardwareMap.get(TouchSensor.class,"horReset");

    }
}
