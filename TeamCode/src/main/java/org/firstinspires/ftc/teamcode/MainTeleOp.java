
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@TeleOp(name = "Main TeleOp")
public class MainTeleOp extends LinearOpMode {
    Limelight3A limelight;

    ArrayList<Integer> IDs = new ArrayList<>();
    //.5=green | 1=p | 0=p
    ArrayList<Double> motifArray = new ArrayList<>(Arrays.asList(.5, 0.0, 1.0, 1.0, .5, 0.0, 1.0, 0.0, .5));

    int id;

    DcMotor intakeMotor;
    Servo goofyAhhhhFrontDoor;
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftLauncher;
    DcMotor rightLauncher;
    DcMotor lift;
    CRServo launchLiftRight;
    CRServo launchLiftLeft;
    Servo backDoor;
    Servo scoop;
    Servo turnTableServo;
    TouchSensor TopBump;
    TouchSensor BottomBump;
    CRServo LED1;
    GoBildaPinpointDriver pinpoint;
    private TouchSensor intakeBump1;
    private TouchSensor intakeBump2;

    int highLauncherSpeed = 2350;//2400;
    int lowLauncherSpeed = 1750;
    int triangleFuncRunning = 0;
    double turnTablePos2 = 0;
    int launcherSpeed = 0;
    int LauncherON = 0;
    int intakeCount = 0;
    int ballCount = 0;
    int ballTrig = 0;

    int motiff = 1;
    int manualMotif = 1;
    double speed = 0;





    ElapsedTime triangleClock = new ElapsedTime();
    ElapsedTime ReKickClock = new ElapsedTime();
    ElapsedTime ScoopClock = new ElapsedTime();
    ElapsedTime LaunchClock = new ElapsedTime();
    ElapsedTime LaunchMotiffClock = new ElapsedTime();
    ElapsedTime intake3BallsClock = new ElapsedTime();

    int LaunchMotiffTrig = 0;
    int LaunchTrig = 0;
    int RekickTrig = 0;
    int scoopTrig = 0;
    int triTrig = 0;
    int LocalTrig = 0;
    int ledTrig = 0;
    int bumpTrig = 0;
    int motifTrig = 1;
    boolean processTrig = true;
    boolean launchAbort = false;
    boolean safeTrig = true;
    boolean resetTrig = false;
    boolean reverseLaunchTrig = true;

    double txMax = 14.700;
    double txMin = 14.400;
    double tyMax = 13.5;
    double tyMin = 12;
    double taMax = .88;
    double taMin = .91;

    Pose2d redParkPose = new Pose2d(60, -60, Math.toRadians(0));
    Pose2d blueParkPose = new Pose2d(60, 60, Math.toRadians(0));


    @Override
    public void runOpMode() {

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        goofyAhhhhFrontDoor = hardwareMap.get(Servo.class, "goofyAhhhhFrontDoor");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftLauncher = hardwareMap.get(DcMotor.class, "leftLauncher");
        rightLauncher = hardwareMap.get(DcMotor.class, "rightLauncher");
        lift = hardwareMap.get(DcMotor.class, "lift");
        launchLiftRight = hardwareMap.get(CRServo.class, "launchLiftRight");
        launchLiftLeft = hardwareMap.get(CRServo.class, "launchLiftLeft");
        backDoor = hardwareMap.get(Servo.class, "backDoor");
        scoop = hardwareMap.get(Servo.class, "scoop");
        turnTableServo = hardwareMap.get(Servo.class, "turnTableServo");
        TopBump = hardwareMap.get(TouchSensor.class, "TopBump");
        BottomBump = hardwareMap.get(TouchSensor.class, "BottomBump");
        DistanceSensor distance = hardwareMap.get(DistanceSensor.class, "distance");
        ColorSensor color_DistanceSensor = hardwareMap.get(ColorSensor.class, "color");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        LED1 = hardwareMap.get(CRServo.class, "Led1");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        intakeBump1 = hardwareMap.get(TouchSensor.class, "intakeBump1");
        intakeBump2 = hardwareMap.get(TouchSensor.class, "intakeBump2");


        // Put initialization blocks here.
        triangleFuncRunning = 0;
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        leftLauncher.setDirection(DcMotor.Direction.REVERSE);
        rightLauncher.setDirection(DcMotor.Direction.FORWARD);
        launchLiftRight.setDirection(CRServo.Direction.REVERSE);
        launchLiftLeft.setDirection(CRServo.Direction.FORWARD);
        leftLauncher.setPower(0);
        rightLauncher.setPower(0);
        leftLauncher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightLauncher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start();
        ledManager("Null");


        waitForStart();
        speed = 0.75;
        backDoor.setPosition(1);
        goofyAhhhhFrontDoor.setPosition(0.5);
        scoop.setPosition(1);
        turnTableServo.setPosition(0.5);
        turnTablePos2 = 0;
        launcherSpeed = (1700 * 28) / 60;
        triangleFuncRunning = 1;

        pinpoint.initialize();

        new MMHS26Lib(hardwareMap, new Pose2d(0, 0, 0), telemetry);

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                telemetry.update();
                pinpoint.update();
                telemetry.addData("x", pinpoint.getEncoderX());
                telemetry.addData("y", pinpoint.getEncoderY());
                telemetry.addData("posX", pinpoint.getPosX(DistanceUnit.MM));
                telemetry.addData("posY", pinpoint.getPosY(DistanceUnit.MM));
                telemetry.addData("headingDeg", pinpoint.getHeading(AngleUnit.DEGREES));
                telemetry.addData("intakeCount",intakeCount);
                telemetry.addData("intakeBump1",intakeBump1.isPressed());

                intakeControl();
                drive();
                turnTablePos();
                timeTriangleFunction();
                backDoorControl();
                timeReKick();
                timeScoop();
                controlLauncher();
                goofyAhhhhFrontDoorControl();
                launcherTiltControl();
                //distanceSensorControl();
                killSwitch();
                localize(0.3, 10);
                lift();
                motifControl();
                timeLaunchMotif(manualMotif, launcherSpeed);
                toClose();
                intake3Balls();
                reverseLaunch();
                park();

                /*

                if (manualMotif != 3) {
                    timeLaunchMotif(manualMotif, launcherSpeed);
                } else {
                    timeLaunchMotif(motiff, launcherSpeed);
                }

                if (motifTrig == 1 && manualMotif == 3) {
                    processLimeLightResults();
                    checkID();
                }

                 */


            }
        }
    }
    private void checkID() {
        for (int idd : IDs) {
            if (idd == 21){
                motiff = 0;
                gamepad1.rumbleBlips(2);
                motifTrig = 0;
            } else if (idd == 22) {
                motiff = 1;
                gamepad1.rumbleBlips(2);
                motifTrig = 0;
            } else if (idd == 23) {
                motiff = 2;
                gamepad1.rumbleBlips(2);
                motifTrig = 0;
            }
        }
    }


    private void intakeControl() {
        if (gamepad1.left_trigger == 1) {
            intakeMotor.setPower(0.8);
            goofyAhhhhFrontDoor.setPosition(1);
        } else if (gamepad1.right_trigger == 1) {
            intakeMotor.setPower(0);
        }
    }

    private void backDoorControl() {
        if (gamepad2.squareWasPressed()) {
            backDoor.setPosition(1);
        }
        if (gamepad2.circleWasPressed()) {
            backDoor.setPosition(0);
        }
    }

    private void launcherTiltControl() {

        if (-0.1 >= gamepad2.right_stick_y && !BottomBump.isPressed()) {
            launchLiftRight.setPower(gamepad2.right_stick_y * 0.35);
            launchLiftLeft.setPower(gamepad2.right_stick_y * 0.35);
        } else if (0.1 <= gamepad2.right_stick_y && !TopBump.isPressed()) {
            launchLiftRight.setPower(gamepad2.right_stick_y * 0.35);
            launchLiftLeft.setPower(gamepad2.right_stick_y * 0.35);
        } else if (0.1 <= gamepad2.right_stick_y && TopBump.isPressed()) {
            launchLiftRight.setPower(0);
            launchLiftRight.setPower(0);
            launchLiftLeft.setPower(0);
            gamepad2.rumbleBlips(1);
        } else if (-0.1 >= gamepad2.right_stick_y && BottomBump.isPressed()) {
            launchLiftRight.setPower(0);
            launchLiftLeft.setPower(0);
            gamepad2.rumbleBlips(1);
        } else {
            launchLiftRight.setPower(0);
            launchLiftLeft.setPower(0);
        }
        if (TopBump.isPressed() && gamepad2.right_stick_y != 0) {
            ledManager("Blue");
            ledTrig = 1;
            bumpTrig = 1;
        } else if  (BottomBump.isPressed() && gamepad2.right_stick_y != 0) {
            ledManager("Blue");
            ledTrig = 1;
            bumpTrig = 1;
        }else if (bumpTrig == 1 && !(BottomBump.isPressed() || TopBump.isPressed())){
            ledManager("Null");
            ledTrig = 0;
            bumpTrig = 0;
        }
    }

    private void goofyAhhhhFrontDoorControl() {
        if (gamepad1.squareWasReleased()) {
            goofyAhhhhFrontDoor.setPosition(1);
        } else if (gamepad1.circleWasReleased()) {
            goofyAhhhhFrontDoor.setPosition(0);
        } else if (gamepad1.crossWasReleased()) {
            goofyAhhhhFrontDoor.setPosition(0.5);
        }
    }

    private void killSwitch() {
        if (gamepad1.touchpadWasPressed()) {
            leftLauncher.setPower(0);
            rightLauncher.setPower(0);
            intakeMotor.setPower(0);
            launchAbort = true;
        }
    }

    private void turnTablePos() {
        if (gamepad2.leftBumperWasPressed() && 0 != goofyAhhhhFrontDoor.getPosition()) {
            turnTablePos2 += 0.5;
            if (1.5 <= turnTablePos2) {
                turnTablePos2 = 0;
            }
            turnTableServo.setPosition(turnTablePos2);
        }
        if (gamepad2.rightBumperWasPressed() && 0 != goofyAhhhhFrontDoor.getPosition()) {
            turnTablePos2 -= 0.5;
            if (-0.5 >= turnTablePos2) {
                turnTablePos2 = 1;
            }
            turnTableServo.setPosition(turnTablePos2);
        }
        if (gamepad2.psWasPressed() && 0 != goofyAhhhhFrontDoor.getPosition()) {
            turnTablePos2 = 0;
            turnTableServo.setPosition(turnTablePos2);
        }
    }

    private void drive() {
        @SuppressWarnings("SuspiciousNameCombination")
        float ControlY = gamepad1.left_stick_x;
        float ControlX = -gamepad1.right_stick_x;
        float ControlRX = -gamepad1.right_stick_y;
        leftFront.setPower(((ControlY - ControlX) + ControlRX) * speed);
        leftBack.setPower((ControlY + ControlX + ControlRX) * speed);
        rightFront.setPower(((ControlY - ControlX) - ControlRX) * speed);
        rightBack.setPower(((ControlY + ControlX) - ControlRX) * speed);
        if (gamepad1.dpad_up) {
            speed = 1;
        } else if (gamepad1.dpad_right) {
            speed = 0.75;
        } else if (gamepad1.dpad_down) {
            speed = 0.35;
        }
    }

    private void timeReKick() {
        if (gamepad2.touchpadWasReleased()) {
            ReKickClock.reset();
            telemetry.addData("Elapsed Time", ReKickClock.seconds());
            RekickTrig = 1;
        }
        if (RekickTrig == 1) {
            if (ReKickClock.seconds() >= 0 && ReKickClock.seconds() <= 0.75) {
                goofyAhhhhFrontDoor.setPosition(0);
                telemetry.update();
            }
            if (ReKickClock.seconds() >= 0.75 && ReKickClock.seconds() <= 1) {
                goofyAhhhhFrontDoor.setPosition(0.5);
                telemetry.update();
                RekickTrig = 0;
            }
        }
    }

    private void timeTriangleFunction() {

        if (gamepad2.triangleWasReleased()) {
            triangleFuncRunning = 1;

            triangleClock.reset();
            telemetry.addData("Elapsed Time", triangleClock.seconds());
            launchAbort = false;
            triTrig = 1;
        }
        if (triTrig == 1 && !launchAbort) {
            if (triangleClock.seconds() >= 0 && triangleClock.seconds() <= 0.5) {
                triangleFuncRunning = 0;
                launchMotorOnTriangle();
                backDoor.setPosition(0);
                telemetry.update();
                ledManager("Alert");
                ledTrig = 1;
            }
            if (triangleClock.seconds() >= 0.5 && triangleClock.seconds() <= 1.5) {
                launchMotorOnTriangle();
                goofyAhhhhFrontDoor.setPosition(0);
                telemetry.update();
            }
            if (triangleClock.seconds() >= 1.5 && triangleClock.seconds() <= 2) {
                launchMotorOnTriangle();
                goofyAhhhhFrontDoor.setPosition(0.5);
                scoop.setPosition(0.5);
                telemetry.update();
            }
            if (triangleClock.seconds() >= 2.5 && triangleClock.seconds() <= 3) {
                launchMotorOnTriangle();
                scoop.setPosition(1);
                telemetry.update();
                triangleFuncRunning = 0;
                triTrig = 0;
                ledManager("Null");
                ledTrig = 0;
            }

        }
    }
    private void timeScoop() {
        if (gamepad2.dpad_up) {
            ScoopClock.reset();
            telemetry.addData("Elapsed Time", ScoopClock.seconds());
            scoopTrig = 1;
        }
        if (scoopTrig == 1) {
            if (ScoopClock.seconds() >= 0 && ScoopClock.seconds() <= 0.5) {
                scoop.setPosition(0.5);
                telemetry.update();
            }
            if (ScoopClock.seconds() >= 0.5 && ScoopClock.seconds() <= 1) {
                scoop.setPosition(1);
                telemetry.update();
                scoopTrig = 0;
            }
        }
    }

    private void controlLauncher() {
        if (gamepad2.left_trigger == 1) {

            ((DcMotorEx) leftLauncher).setVelocity(launcherSpeed);
            ((DcMotorEx) rightLauncher).setVelocity(launcherSpeed);

            LauncherON = 1;
        } else if (gamepad2.right_trigger == 1) {
            ((DcMotorEx) leftLauncher).setVelocity(0);
            ((DcMotorEx) rightLauncher).setVelocity(0);
            LauncherON = 0;
        }
        if (gamepad2.dpadLeftWasPressed()) {
            launcherSpeed = (lowLauncherSpeed * 28) / 60;
            gamepad2.rumbleBlips(1);
            if (1 == LauncherON) {

                ((DcMotorEx) leftLauncher).setVelocity(launcherSpeed);
                ((DcMotorEx) rightLauncher).setVelocity(launcherSpeed);
            }
        } else if (gamepad2.dpadRightWasReleased()) {
            launcherSpeed = (highLauncherSpeed * 28) / 60;
            gamepad2.rumbleBlips(2);
            if (1 == LauncherON) {

               ((DcMotorEx) leftLauncher).setVelocity(launcherSpeed);
               ((DcMotorEx) rightLauncher).setVelocity(launcherSpeed);
            }
        }
    }

    private void launchMotorOnTriangle() {
       ((DcMotorEx) leftLauncher).setVelocity(launcherSpeed * Math.abs(triangleFuncRunning - 1));
       ((DcMotorEx) rightLauncher).setVelocity(launcherSpeed * Math.abs(triangleFuncRunning - 1));

    }
    public void localize(double localizerMotorPower, int sleepTimeMilli) {
        if (gamepad1.ps) {
            LocalTrig = 1;
            //boolean localizing = true;

            LLResult result = limelight.getLatestResult();
            double tx;
            double ty;
            double ta;
            if (result != null && result.isValid()) {
                tx = result.getTx(); // How far left or right the target is (degrees)
                ty = result.getTy(); // How far up or down the target is (degrees)
                ta = result.getTa(); // How big the target looks (0%-100% of the image)

                telemetry.addData("Target X", tx);
                telemetry.addData("Target Y", ty);
                telemetry.addData("Target Area", ta);
                telemetry.update();

                if (tx < txMin) {
                    // turn right
                    turnRight(localizerMotorPower, sleepTimeMilli);
                    ledManager("Clear");
                    LocalTrig = 1;
                } else if (tx > txMax) {
                    //turn left
                    turnLeft(localizerMotorPower, sleepTimeMilli);
                    ledManager("Clear");
                    LocalTrig = 1;
                } else if (ty < tyMin) {
                    //strafe in a direction (i think )
                    strafeRight(localizerMotorPower, sleepTimeMilli);
                    ledManager("Clear");
                    LocalTrig = 1;
                } else if (ty > tyMax) {
                    //strafe in a direction (i think left)
                    strafeLeft(localizerMotorPower, sleepTimeMilli);
                    ledManager("Clear");
                    LocalTrig = 1;
                } else if (ta > taMax) {
                    //move backward
                    moveBackward(localizerMotorPower, sleepTimeMilli);
                    ledManager("Clear");
                    LocalTrig = 1;
                } else if (ta < taMin) {
                    //move Forward
                    moveForward(localizerMotorPower, sleepTimeMilli);
                    ledManager("Clear");
                    LocalTrig = 1;
                } else {
                    telemetry.addData("done", 0);
                    ledManager("Good");
                    LocalTrig = 1;
                }

            } else {
                telemetry.addData("Limelight", "No Targets");
                ledManager("Error");
                LocalTrig = 1;
                halt();
            }
        } else if (LocalTrig == 1){
            ledManager("Null");
            LocalTrig = 0;
        }
    }
    public void strafeLeft(double Speed, int time) {
        leftBack.setPower(Speed);
        leftFront.setPower(-Speed);
        rightBack.setPower(-Speed);
        rightFront.setPower(Speed);
        sleep(time);
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }
    public void strafeRight(double Speed, int time) {
        leftBack.setPower(-Speed);
        leftFront.setPower(Speed);
        rightBack.setPower(Speed);
        rightFront.setPower(-Speed);
        sleep(time);
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }
    public void turnRight(double Speed, int time) {
        leftBack.setPower(Speed);
        leftFront.setPower(Speed);
        rightBack.setPower(-Speed);
        rightFront.setPower(-Speed);
        sleep(time);
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }
    public void turnLeft(double Speed, int time) {
        leftBack.setPower(-Speed);
        leftFront.setPower(-Speed);
        rightBack.setPower(Speed);
        rightFront.setPower(Speed);
        sleep(time);
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }
    public void moveBackward(double Speed, int time) {
        leftBack.setPower(Speed);
        leftFront.setPower(Speed);
        rightBack.setPower(Speed);
        rightFront.setPower(Speed);
        sleep(time);
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }
    public void moveForward(double Speed, int time) {
        leftBack.setPower(-Speed);
        leftFront.setPower(-Speed);
        rightBack.setPower(-Speed);
        rightFront.setPower(-Speed);
        sleep(time);
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }
    public void halt() {
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }

    private void lift(){
        lift.setPower(gamepad2.left_stick_y);
        /*
        if (!liftLimit.isPressed()) {
            lift.setPower(gamepad2.right_stick_y);
        } else {
            lift.setPower(0);
            ledManager("Good");
        }

         */

    }
    private void ledManager(String type){
        switch (type) {
            case "Clear":
                LED1.setPower(.5); //White

                break;
            case "Good":
                LED1.setPower(0); //Green

                break;
            case "Warn":
                LED1.setPower(-.25); //Yellow

                break;
            case "Alert":
                LED1.setPower(-.35); //Orange

                break;
            case "Error":
                LED1.setPower(-0.44); //red

                break;
            case "Null":
                LED1.setPower(-.6); //Blank

                break;
            case "Match Alert":
                LED1.setPower(0); //Purple

                break;
            case "Blue":
                LED1.setPower(0.216); //Blue

                break;
            case "Purple":
                LED1.setPower(0.415); //Purple

                break;
            case "Pink":
                LED1.setPower(0.275); //Pink

                break;
            default:
                telemetry.addData("Led Manager Error", "Wrong or Invalid Input");
                break;
        }
    }

    private void timeLaunchMotif(int motiff, double launcherSpeedd) {

        if (gamepad2.crossWasReleased()) {
            LaunchMotiffClock.reset();
            telemetry.addData("Elapsed Time", LaunchMotiffClock.seconds());
            LaunchMotiffTrig = 1;
            launchAbort = false;
            telemetry.update();
        }
        if (LaunchMotiffTrig == 1 && !launchAbort) {
            if (LaunchMotiffClock.seconds() >= 0 && LaunchMotiffClock.seconds() <= 0.75) {
                ledManager("Alert");
                launchMotorOn(launcherSpeedd);
                backDoor.setPosition(0);
                turnTableServo.setPosition(0); //motifArray.get(motiff*3)
                telemetry.update();
            }
            if (LaunchMotiffClock.seconds() >= 0.75 && LaunchMotiffClock.seconds() <= 1) {

                //backDoor.setPosition(0);
                telemetry.update();
            }
            if (LaunchMotiffClock.seconds() >= 1 && LaunchMotiffClock.seconds() <= 1.75) {
                goofyAhhhhFrontDoor.setPosition(0);
                telemetry.update();

            }
            if (LaunchMotiffClock.seconds() >= 1.75 && LaunchMotiffClock.seconds() <= 1.85) {
                goofyAhhhhFrontDoor.setPosition(0.5);
                telemetry.update();

            }
            if (LaunchMotiffClock.seconds() >= 1.85 && LaunchMotiffClock.seconds() <= 2.35) {
                //backDoor.setPosition(1);
                goofyAhhhhFrontDoor.setPosition(0.5);
                scoop.setPosition(0.5);

                turnTableServo.setPosition(.5); //motifArray.get((motiff*3)+1)
                telemetry.update();
            }


            if (LaunchMotiffClock.seconds() >= 2.35 && LaunchMotiffClock.seconds() <= 2.6) {
                scoop.setPosition(1);
                backDoor.setPosition(0);
                telemetry.update();
            }
            if (LaunchMotiffClock.seconds() >= 2.6 && LaunchMotiffClock.seconds() <= 3.35) {
                goofyAhhhhFrontDoor.setPosition(0);
                telemetry.update();

            }
            if (LaunchMotiffClock.seconds() >= 3.35 && LaunchMotiffClock.seconds() <= 3.45) {
                goofyAhhhhFrontDoor.setPosition(0.5);
                telemetry.update();

            }
            if (LaunchMotiffClock.seconds() >= 3.45 && LaunchMotiffClock.seconds() <= 3.95) {
                //backDoor.setPosition(1);
                goofyAhhhhFrontDoor.setPosition(0.5);
                scoop.setPosition(0.5);

                turnTableServo.setPosition(1); //motifArray.get((motiff*3)+2
                telemetry.update();
            }


            if (LaunchMotiffClock.seconds() >= 3.95 && LaunchMotiffClock.seconds() <= 4.2) {
                scoop.setPosition(1);
                backDoor.setPosition(0);
                telemetry.update();
            }
            if (LaunchMotiffClock.seconds() >= 4.2 && LaunchMotiffClock.seconds() <= 4.95) {
                goofyAhhhhFrontDoor.setPosition(0);
                telemetry.update();

            }
            if (LaunchMotiffClock.seconds() >= 4.95 && LaunchMotiffClock.seconds() <= 5.05) {
                goofyAhhhhFrontDoor.setPosition(0.5);
                telemetry.update();

            }
            if (LaunchMotiffClock.seconds() >= 5.05 && LaunchMotiffClock.seconds() <= 5.55) {

                goofyAhhhhFrontDoor.setPosition(0.5);
                scoop.setPosition(0.5);

                telemetry.update();
            }
            if (LaunchMotiffClock.seconds() >= 5.55 && LaunchMotiffClock.seconds() <= 6.05) {
                scoop.setPosition(1);
                backDoor.setPosition(1);
                turnTableServo.setPosition(0);
                launchMotorOff();
                telemetry.update();
                ledManager("Null");
                LaunchMotiffTrig = 0;
            }
        }

    }


    private int processLimeLightResults() {
        double tx;
        double ty;
        double ta;

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            // Get the list of ALL detected fiducials (AprilTags)
            List<LLResultTypes.FiducialResult> fiducialList = result.getFiducialResults();

            tx = result.getTx();
            ty = result.getTy(); // How far up or down the target is (degrees)
            ta = result.getTa(); // How big the target looks (0%-100% of the image)

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
            telemetry.addData("Target Area", ta);
            telemetry.update();

            if (!fiducialList.isEmpty()) {
                telemetry.addData("Detections Found", fiducialList.size());
                telemetry.update();

                // Iterate through each detected tag
                for (LLResultTypes.FiducialResult fiducial : fiducialList) {
                    id = fiducial.getFiducialId();
                    IDs.add(id);
                    telemetry.addData("Tag ID", id);
                    telemetry.update();
                }
                processTrig = false;
            } else {
                telemetry.addData("Detections Found", "None");
                telemetry.update();
            }
        } else {
            telemetry.addData("Limelight Data", "Invalid or Stale");
            assert result != null;
            telemetry.addData("Staleness", result.getStaleness());
            telemetry.update();
        }
        telemetry.update();

        return id;
    }
    public void motifControl() {
        if (gamepad2.optionsWasPressed()) {
            manualMotif++;
            if (manualMotif > 2) {
                manualMotif = 0;
            }
            /* Is there a reason for this?
            if (manualMotif == 3) {
                motifTrig = 1;
            }
             */

            gamepad2.rumbleBlips(manualMotif+1);
        }
    }

    private void launchMotorOn(double launcherSpeedd) {
        ((DcMotorEx) leftLauncher).setVelocity(launcherSpeedd);
        ((DcMotorEx) rightLauncher).setVelocity(launcherSpeedd);


    }
    private void launchMotorOff() {
        ((DcMotorEx) leftLauncher).setVelocity(0);
        ((DcMotorEx) rightLauncher).setVelocity(0);

    }
    private void toClose() {
        if (gamepad1.options){
            MMHS26Lib.roadRunner.spline.splineToLinearHeading(-28, 24,  Math.toRadians(25), 0, MMHS26Lib.Limelight.poseLimelight(false));
        }
    }
    private void reverseLaunch(){

        if (gamepad1.shareWasPressed()){
            leftLauncher.setPower(-.3);
            rightLauncher.setPower(-.3);
            reverseLaunchTrig = true;
        } else if (gamepad1.shareWasReleased()){
            leftLauncher.setPower(0);
            rightLauncher.setPower(0);

        }
    }
    private void park() {
        if(gamepad1.optionsWasReleased()){
            MMHS26Lib.roadRunner.spline.splineToLinearHeading(-28, 24, Math.toRadians(25), 0, redParkPose);
        }
        /*
        if(gamepad1.shareWasReleased()){
            MMHS26Lib.roadRunner.spline.splineToLinearHeading(-28, 24, Math.toRadians(25), 0, blueParkPose);
        }

         */
    }

    public void intake3Balls() {


        if (gamepad1.triangleWasPressed()) {

            turnTableServo.setPosition(0);
            //goofyAhhhhFrontDoor.setPosition(1);
            intakeOn();
            intake3BallsClock.reset();
            intakeCount = 1;
            ballCount = 0;
            resetTrig = true;
            safeTrig = false;
        }

        if (intakeCount == 1 && (!intakeBump1.isPressed() || intakeBump2.isPressed())) {
            ballTrig = 1;
            ballCount = 1;
            intake3BallsClock.reset();
        } else if (intakeCount == 2 && (!intakeBump1.isPressed() || intakeBump2.isPressed())) {
            ballTrig = 1;
            ballCount = 2;
            intake3BallsClock.reset();
        } else if (intakeCount == 3 && (!intakeBump1.isPressed() || intakeBump2.isPressed())) {
            ballTrig = 1;
            ballCount = 3;
            intake3BallsClock.reset();
        }

        if (ballCount == 1 && gamepad1.triangle == true && ballTrig == 1) {


            if (intake3BallsClock.seconds() >= 0 && intake3BallsClock.seconds() <= .500 && gamepad1.triangle == true) {
                goofyAhhhhFrontDoor.setPosition(0);
            }

            if (intake3BallsClock.seconds() >= .500 && intake3BallsClock.seconds() <= .510 && gamepad1.triangle == true) {
                goofyAhhhhFrontDoor.setPosition(.5);
            }

            if (intake3BallsClock.seconds() >= .510 && intake3BallsClock.seconds() <= .550 && gamepad1.triangle == true) {
                turnTableServo.setPosition(0.5);
                goofyAhhhhFrontDoor.setPosition(1);
                intakeCount = 2;
                ballTrig = 0;
            }
        } else if (ballCount == 2 && gamepad1.triangle == true && ballTrig == 1) {

            if (intake3BallsClock.seconds() >= 0 && intake3BallsClock.seconds() <= .500 && gamepad1.triangle == true) {
                goofyAhhhhFrontDoor.setPosition(0);
            }

            if (intake3BallsClock.seconds() >= .500 && intake3BallsClock.seconds() <= .510 && gamepad1.triangle == true) {
                goofyAhhhhFrontDoor.setPosition(.5);
            }

            if (intake3BallsClock.seconds() >= .510 && intake3BallsClock.seconds() <= .550 && gamepad1.triangle == true) {
                turnTableServo.setPosition(1);
                goofyAhhhhFrontDoor.setPosition(1);
                intakeCount = 3;
                ballTrig = 0;
            }


        } else if (ballCount == 3 && gamepad1.triangle == true && ballTrig == 1) {

            if (intake3BallsClock.seconds() >= 0 && intake3BallsClock.seconds() <= .500 && gamepad1.triangle == true) {
                goofyAhhhhFrontDoor.setPosition(0);
            }

            if (intake3BallsClock.seconds() >= .500 && intake3BallsClock.seconds() <= .510 && gamepad1.triangle == true) {
                goofyAhhhhFrontDoor.setPosition(.5);
            }

            if (intake3BallsClock.seconds() >= .510 && intake3BallsClock.seconds() <= .550 && gamepad1.triangle == true) {
                turnTableServo.setPosition(1);
                goofyAhhhhFrontDoor.setPosition(.5);
                intakeCount = 3;
                ballTrig = 0;
                intakeOff();
            }



        }
        if (gamepad1.triangleWasReleased()) {
            intakeCount = 0;
            goofyAhhhhFrontDoor.setPosition(.5);
            intakeOff();
            safeTrig = true;
            resetTrig = false;
        }
        /*


        if (intakeCount == 1 && (!intakeBump1.isPressed() || intakeBump2.isPressed()) && gamepad1.triangle == true) {
            intake3BallsClock.reset();
            if (intake3BallsClock.seconds() >= 0 && intake3BallsClock.seconds() <= .500 && gamepad1.triangle == true) {
                goofyAhhhhFrontDoor.setPosition(0);
            }

            if (intake3BallsClock.seconds() >= .500 && intake3BallsClock.seconds() <= .510 && gamepad1.triangle == true) {
                goofyAhhhhFrontDoor.setPosition(.5);
            }

            if (intake3BallsClock.seconds() >= .510 && intake3BallsClock.seconds() <= .550 && gamepad1.triangle == true) {
                turnTableServo.setPosition(0.5);
                goofyAhhhhFrontDoor.setPosition(1);
                intakeCount = 2;
            }

        } else if (intakeCount == 2 && (!intakeBump1.isPressed() || intakeBump2.isPressed()) && gamepad1.triangle == true) {
            intake3BallsClock.reset();
            if (intake3BallsClock.seconds() >= 0 && intake3BallsClock.seconds() <= .500 && gamepad1.triangle == true) {
                goofyAhhhhFrontDoor.setPosition(0);
            }

            if (intake3BallsClock.seconds() >= .500 && intake3BallsClock.seconds() <= .510 && gamepad1.triangle == true) {
                goofyAhhhhFrontDoor.setPosition(.5);
            }

            if (intake3BallsClock.seconds() >= .510 && intake3BallsClock.seconds() <= .550 && gamepad1.triangle == true) {
                turnTableServo.setPosition(1);
                goofyAhhhhFrontDoor.setPosition(1);
                intakeCount = 3;
            }
        } else if (intakeCount == 3 && (!intakeBump1.isPressed() || intakeBump2.isPressed()) && gamepad1.triangle == true) {
            intake3BallsClock.reset();
            if (intake3BallsClock.seconds() >= 0 && intake3BallsClock.seconds() <= .500 && gamepad1.triangle == true) {
                goofyAhhhhFrontDoor.setPosition(0);
            }

            if (intake3BallsClock.seconds() >= .500 && intake3BallsClock.seconds() <= .510 && gamepad1.triangle == true) {
                goofyAhhhhFrontDoor.setPosition(.5);
            }

            if (intake3BallsClock.seconds() >= .510 && intake3BallsClock.seconds() <= .550 && gamepad1.triangle == true) {
                turnTableServo.setPosition(1);
                goofyAhhhhFrontDoor.setPosition(.5);
                intakeCount = 0;
                intakeOff();
            }


        }

            if (gamepad1.triangle == false && resetTrig == true) {
                intakeCount = 0;
                goofyAhhhhFrontDoor.setPosition(.5);
                intakeOff();
                safeTrig = true;
                resetTrig = false;
            }

         */


    }



        public void intakeOn () {
            intakeMotor.setPower(0.8);
        }

        public void intakeOff () {
            intakeMotor.setPower(0);

        }

        public void moveForwardTics ( double Speed, double tic){
            pinpoint.update();
            double xvalue = pinpoint.getEncoderX();
            while (xvalue - pinpoint.getEncoderX() <= tic) {
                pinpoint.update();
                leftBack.setPower(-Speed);
                leftFront.setPower(-Speed);
                rightBack.setPower(-Speed);
                rightFront.setPower(-Speed);
            }
            leftBack.setPower(0);
            leftFront.setPower(0);
            rightBack.setPower(0);
            rightFront.setPower(0);
        }

    }


   
