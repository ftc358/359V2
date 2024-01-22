package org.firstinspires.ftc.teamcode.teleop;


import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.*;
import static java.lang.Thread.sleep;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.threadopmode.*;
@TeleOp(name="LibrA+ TeleOP 2.0a", group="TeleOP")

public class ILT_TeleOP extends ThreadOpMode{
    private Gamepad currentGamepad1 = new Gamepad();
    private Gamepad currentGamepad2 = new Gamepad();
    private Gamepad previousGamepad1 = new Gamepad();
    private Gamepad previousGamepad2 = new Gamepad();
    //Vars for Drive Thread
    private double x, y, turn, theta, power,sin,cos,lf,lb,rf,rb,max;
    private double turnScale = 1.0;
    private double powerScale = 1.0;
    //Vars for Motor Extension thread
    private double horExtPwr;
    private double liftPwr;
    private double intakePwr;
    //Vars for Sensor Thread
    private double backDistanceIN;
    private boolean horExtRetract;
    //Vars for Pixel Thread
    private double claw1Pos = 0;
    private double claw2Pos = 0;
    private double wristPos = 0;
    private double diffy1Pos = 0;
    private double diffy2Pos = 0;
    private double intakeWrist = 0;
    private double intakePivot = 0;
    //Vars for Led Thread
    private int ledPattern = 0;
    private RevBlinkinLedDriver.BlinkinPattern currentPattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
    private RevBlinkinLedDriver.BlinkinPattern previousPattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;

    private int grabState = 0;
    private int wristState = 0;
    private int diffyState = 0;

    private int intakeTransfer = 0;
    long halt0, halt1, halt2, halt3, halt4, halt5;
    private boolean skiing = false;

    private boolean desire;



    @Override
    public void mainInit() {
        Robot Robot = new Robot(hardwareMap);

        TaskThread driveThread = new TaskThread(new TaskThread.Actions() {
            @Override
            public void loop() {
                //Mecanum Drive Code From https://www.youtube.com/watch?v=gnSW2QpkGXQ
                x = currentGamepad1.left_stick_x;
                y = -currentGamepad1.left_stick_y;
                turn = currentGamepad1.right_stick_x*turnScale;
                theta = Math.atan2(y,x);
                power = Math.hypot(x,y)*powerScale;


                sin = Math.sin(theta-Math.PI/4);
                cos = Math.cos(theta-Math.PI/4);
                max = Math.max(Math.abs(sin),Math.abs(cos));

                lf = power*cos/max +turn;
                rf = power*sin/max -turn;
                lb = power*sin/max +turn;
                rb = power*cos/max -turn;

                if ((power + Math.abs(turn)) > 1) {
                    lf  /= power + Math.abs(turn);
                    rf  /= power + Math.abs(turn);
                    lb  /= power + Math.abs(turn);
                    rb  /= power + Math.abs(turn);
                }

                Robot.leftFront.setPower(lf);
                Robot.leftBack.setPower(lb);
                Robot.rightFront.setPower(rf);
                Robot.rightBack.setPower(rb);
            }
        });

        TaskThread sensorThread = new TaskThread(new TaskThread.Actions() {
            @Override
            public void loop() {
                backDistanceIN = Robot.backDistance.getDistance(DistanceUnit.INCH);
                horExtRetract = Robot.horReset.isPressed();
            }
        });

        TaskThread horExtThread = new TaskThread(new TaskThread.Actions() {
            @Override
            public void loop() {
                //Horizontal Extension Triggering
                if ((currentGamepad2.left_trigger>0.2)){
                    desire = false;
                    horExtPwr = currentGamepad2.left_trigger;

                }
                else if (currentGamepad2.left_bumper && !horExtRetract){
                    desire = false;
                    horExtPwr = -1;
                }else if (currentGamepad2.left_bumper)
                {
                    desire = false;
                    horExtPwr = -0.2;
                }else if (horExtRetract||desire){
                    desire = true;
                    horExtPwr = -0.2;
                }else if (!desire){
                    horExtPwr = 0;
                }else {
                    horExtPwr = 0;
                }

                //lift Trigerring
                if (currentGamepad2.right_trigger>0.1){
                    liftPwr = currentGamepad2.right_trigger;
                }else if (currentGamepad2.right_bumper){
                    liftPwr = -1;
                }else{
                    liftPwr = 0;
                }

                if (currentGamepad2.left_trigger>0 || ((intakeTransfer == 1 || intakeTransfer ==2)&&!skiing)){
                    intakePwr = 1;
                }else if (currentGamepad2.dpad_left || (wristState ==1)){
                    intakePwr = -1;
                }else{
                    intakePwr = 0;
                }



                //Set Power to Motor
                Robot.horExt.setPower(horExtPwr);
                Robot.lift.setPower(liftPwr);
                Robot.intake.setPower(intakePwr);
            }
        });


        TaskThread pixelMovement = new TaskThread(new TaskThread.Actions() {
            @Override
            public void loop() {
                switch (diffyState){
                    case 0: //Home position for Driving
                        wristState = 0;
                        intakeTransfer = 0;
                        break;
                    case 1:
                        intakeTransfer = 1;
                        halt0 = 0;
                        break;
                    case 2:
                        intakeTransfer = 2;

                        if (halt0 == 0){
                            halt0 = System.currentTimeMillis();
                        }
                        if ((System.currentTimeMillis()-halt0)>800) {
                            diffyState = 3;
                        }
                        halt1 = 0;

                        break;
                    case 3:
                        wristState = 1;
                        if (halt1 == 0){
                            halt1 = System.currentTimeMillis();
                        }
                        if ((System.currentTimeMillis()-halt1)>700) {
                            skiing = true;
                            diffyState = 4;

                        }
                        halt2 = 0;
                        break;
                    case 4:
                        if (halt2 == 0){
                            halt2 = System.currentTimeMillis();
                        }
                        if ((System.currentTimeMillis()-halt2)>500) {
                            skiing = false;
                            wristState = 2;

                        }
                        break;
                    case 5:
                        wristState = 3;
                        break;
                    case 6:
                        wristState = 4;
                        intakeTransfer = 0;
                        break;
                }





                switch (grabState){
                    case 0: //Not Grabbing
                        claw1Pos = clawReleasePos;
                        claw2Pos = clawReleasePos;
                        break;
                    case 1: //Grabbing
                        claw1Pos = clawGrabPos;
                        claw2Pos = clawGrabPos;
                        break;
                }

                switch (wristState){
                    case 0: // Hold Position
                        diffy1Pos = diffyHoldPos;
                        diffy2Pos = diffyHoldPos;
                        wristPos = wristHoldPos;
                        grabState = 0;
                        break;

                    case 1: // Intake Position
                        grabState = 0;
                        wristPos = wristIntakePos;
                        diffy1Pos = diffyIntakePos;
                        diffy2Pos = diffyIntakePos;
                        break;

                    case 2: // Armed Position
                        grabState = 1;
                        wristPos = wristHoldPos;
                        diffy1Pos = diffyHoldPos;
                        diffy2Pos = diffyHoldPos;
                        break;

                    case 3: //Dropping Position
                        wristPos = wristDropPos+gamepad1.right_trigger*wristInvK_Scale;
                        diffy1Pos = diffyDropPos+gamepad1.right_trigger*diffyInvK_Scale;
                        diffy2Pos = diffyDropPos+gamepad1.right_trigger*diffyInvK_Scale;
                        break;
                    case 4: //Drops
                        grabState = 0;
                }


                switch (intakeTransfer){
                    case 0://Ground Float to prevent driving impediment
                        intakePivot = intakePivotDriving;
                        intakeWrist = intakeWristDriving;
                        break;
                    case 1://Ground Tangent Intake
                        intakePivot = intakePivotIntake;
                        intakeWrist = intakeWristIntake;
                        break;
                    case 2://Transfers
                        intakePivot = intakePivotTransfer;
                        intakeWrist = intakeWristTransfer;
                        break;
                }

                Robot.claw1.setPosition(claw1Pos);
                Robot.claw2.setPosition(claw2Pos);
                Robot.wrist1.setPosition(wristPos);
                Robot.wrist2.setPosition(wristPos);
                Robot.intakePivot1.setPosition(intakePivot);
                Robot.intakePivot2.setPosition(intakePivot);
                Robot.intakeWrist.setPosition(intakeWrist);
                Robot.diffy1.setPosition(diffy1Pos);
                Robot.diffy2.setPosition(diffy2Pos);
            }
        });

        TaskThread ledThread = new TaskThread(new TaskThread.Actions() {
            @Override
            public void loop() {
                if (gamepad1.back){
                    ledPattern = (ledPattern+1)%10;
                }
                switch (ledPattern){
                    case 0:
                        currentPattern = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE;

                        break;
                    case 1:
                        currentPattern = RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE;

                        break;
                    case 2:
                        currentPattern = RevBlinkinLedDriver.BlinkinPattern.AQUA;

                        break;
                    case 3:
                        currentPattern = RevBlinkinLedDriver.BlinkinPattern.LARSON_SCANNER_RED;
                        break;
                    case 4:
                        currentPattern = RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_RAINBOW_PALETTE;

                        break;
                    case 5:
                        currentPattern = RevBlinkinLedDriver.BlinkinPattern.BREATH_RED;

                        break;
                    case 6:
                        currentPattern = RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED;

                        break;
                    case 7:
                        currentPattern = RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE;

                        break;
                    case 8:
                        currentPattern = RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE;

                        break;
                    case 9:
                        currentPattern = RevBlinkinLedDriver.BlinkinPattern.VIOLET;

                        break;
                    case 10:
                        currentPattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;

                        break;


                }

                if (!currentPattern.equals(previousPattern)){
                    Robot.led.setPattern(currentPattern);
                    previousPattern = currentPattern;
                }
            }});

        TaskThread templateThread = new TaskThread(new TaskThread.Actions() {
            @Override
            public void loop() {
            }});

        registerThread(driveThread);
        registerThread(sensorThread);
        registerThread(horExtThread);
        registerThread(pixelMovement);
        registerThread(ledThread);
    }

    @Override
    public void mainLoop() {
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        if (currentGamepad2.a && !previousGamepad2.a){
            diffyState = (diffyState + 1)%7;}



        telemetry.addData("grabState",grabState);
        telemetry.addData("fuck", skiing);
        telemetry.addData("halt1",halt1);
        telemetry.addData("wristState",wristState);
        telemetry.addData("intakepivot",gamepad1.left_trigger);//0.67
        telemetry.addData("intakeWrist",gamepad1.right_trigger);//0.70
        telemetry.addData("diffyPos",diffy1Pos);
        telemetry.addData("BackDistance Reading (in) ",backDistanceIN);
        telemetry.update();

        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);

    }


}
