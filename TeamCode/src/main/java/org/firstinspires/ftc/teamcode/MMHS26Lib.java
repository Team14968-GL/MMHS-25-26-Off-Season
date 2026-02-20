package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.roadRunner.MecanumDrive;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@SuppressWarnings({"unused", "UnusedReturnValue"})
public class MMHS26Lib {
    //Hardware variables
    private static DcMotor leftBack, rightBack, leftFront, rightFront, leftLauncher, rightLauncher, intakeMotor, lift;
    private static CRServo launchLiftRight, launchLiftLeft;
    private static Servo scoop, turnTableServo, backDoor, frontDoor;
    private static GoBildaPinpointDriver pinpoint;
    private static Limelight3A limelight;
    @SuppressWarnings("FieldCanBeLocal")
    private static TouchSensor topBump, bottomBump, intakeBump1, intakeBump2;
    private static ArrayList<CRServo> leds;

    //Constants
    public static final double ticPerIn = 254.7;
    public static ElapsedTime runtime26Lib = new ElapsedTime();


    //Internal variables
    private static HardwareMap hwMap;
    private static Pose2d startPose;
    private static Telemetry telemetry;

    //External Variables
    public static int count = 0;

    public MMHS26Lib(HardwareMap hardwareMap, Pose2d initPose, Telemetry initTelemetry) {
        //Drive Definitions
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        //Intake Definitions
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        frontDoor = hardwareMap.get(Servo.class, "goofyAhhhhFrontDoor");
        intakeBump1 = hardwareMap.get(TouchSensor.class, "intakeBump1");
        intakeBump2 = hardwareMap.get(TouchSensor.class, "intakeBump2");
        //Launcher Definitions
        leftLauncher = hardwareMap.get(DcMotor.class, "leftLauncher");
        rightLauncher = hardwareMap.get(DcMotor.class, "rightLauncher");
        launchLiftRight = hardwareMap.get(CRServo.class, "launchLiftRight");
        launchLiftLeft = hardwareMap.get(CRServo.class, "launchLiftLeft");
        topBump = hardwareMap.get(TouchSensor.class, "TopBump");
        bottomBump = hardwareMap.get(TouchSensor.class, "BottomBump");
        backDoor = hardwareMap.get(Servo.class, "backDoor");
        scoop = hardwareMap.get(Servo.class, "scoop");
        //Lift/Skis Definition
        lift = hardwareMap.get(DcMotor.class, "lift");
        //Turntable Definition
        turnTableServo = hardwareMap.get(Servo.class, "turnTableServo");
        //Drive Config
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //Intake Config
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        //Launcher Config
        leftLauncher.setDirection(DcMotor.Direction.REVERSE);
        rightLauncher.setDirection(DcMotor.Direction.FORWARD);
        leftLauncher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightLauncher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchLiftRight.setDirection(CRServo.Direction.REVERSE);
        launchLiftLeft.setDirection(CRServo.Direction.FORWARD);
        //Lift/Skis Config
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //Odometry Config
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.initialize(); //Initializes odometry for use in code
        pinpoint.update();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, initPose.position.x, initPose.position.y, AngleUnit.DEGREES, Math.toDegrees(initPose.heading.toDouble())));
        pinpoint.setHeading(initPose.heading.toDouble(), AngleUnit.RADIANS);
        pinpoint.update();
        //LED Config
        CRServo LED1 = hardwareMap.get(CRServo.class, "Led1");
        leds = new ArrayList<>(Arrays.asList(null, LED1)); //creates a list of LEDs for ledManager to use
        //Limelight Config/Setup
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); //Sets the config the limelight should use
        limelight.setPollRateHz(100); //Limelight data polling rate
        limelight.start(); //Initializes limelight for use in code

        //Internal Data (DO NOT TOUCH)
        hwMap = hardwareMap;
        initTelemetry.addData("HW Map Initialized", hwMap);
        initTelemetry.update();

        startPose = initPose;
        initTelemetry.addData("Start Pose Initialized", initPose);
        initTelemetry.update();

        telemetry = initTelemetry;
        initTelemetry.addData("Internal Telemetry Initialized", hwMap);
        initTelemetry.update();

        telemetry.addData("Initialization Finished", true);
        telemetry.update();
    }

    //Optional flag(s) to enable internal debugging tools
    @Config
    public static class debug {
        //debug telemetry has to be static to appear on ftc dashboard
        public static boolean debugTelemetry = false;
    }

    //Sleep function taken from LinearOpMode as extending LinearOpMode doesn't work on static classes
    private static void sleep(long milliseconds) {try {Thread.sleep(milliseconds);} catch (InterruptedException e) {Thread.currentThread().interrupt();}}

    //Gets the robots current position on the field
    public static Pose2d currentPose() {
        pinpoint.update();
        return new Pose2d(new Vector2d(pinpoint.getPosX(DistanceUnit.INCH) + startPose.position.x, pinpoint.getPosY(DistanceUnit.INCH) + startPose.position.y), pinpoint.getHeading(AngleUnit.RADIANS) + startPose.heading.toDouble());
    }
    public static class conversions {
        public conversions() {super();}

        public static class pose {
            public pose() {super();}

            //Converts FTC Pose2D to RoadRunner Pose2d
            public static Pose2d Pose2DToPose2d(Pose2D Pose2D) {return new Pose2d(new Vector2d(Pose2D.getX(DistanceUnit.INCH), Pose2D.getY(DistanceUnit.INCH)), Pose2D.getHeading(AngleUnit.DEGREES));}

            //Converts RoadRunner Pose2d to FTC Pose2D
            public static Pose2D Pose2dToPose2D(Pose2d Pose2d) {return new Pose2D(DistanceUnit.INCH, Pose2d.position.x, Pose2d.position.y, AngleUnit.DEGREES, Pose2d.heading.log());}

            //Converts FTC Pose3D to FTC Pose2D
            public static Pose2D Pose3DToPose2D(Pose3D pose3D) {return new Pose2D(pose3D.getPosition().unit, pose3D.getPosition().x, pose3D.getPosition().y, AngleUnit.DEGREES, pose3D.getOrientation().getYaw(AngleUnit.DEGREES));}

            //Converts FTC Pose3D to RoadRunner Pose2d
            public static Pose2d Pose3DtoPose2d(Pose3D pose3D) {return Pose2DToPose2d(Pose3DToPose2D(pose3D));}
        }
        public static class units {
            public units() {super();}

            //Converts millimeters to inches
            public static double millimeterToInch(double millimeter) {return (millimeter / 25.4);}

            //Converts inches to millimeters
            public static double inchToMillimeter(double inch) {return (inch * 25.4);}
        }
    }

    //Class for managing basic functions relating to movement
    public static class motion {
        public motion() {super();}
        public static void mecanumDrive(double X, double Y, double R, double maxSpeed, long T){
            /// X, x movement; Y, y movement; R, turning; and maxSpeed all must be considered as percentages with T representing the time in milliseconds that the robot should move
            //Ensures Y is within the range of -1 to 1
            double controlY = Y;
            if (controlY > 1){
                controlY = 1;
            } else if (controlY < 0){
                controlY = 0;
            }
            //Ensures X is within the range of -1 to 1
            double controlX = -X;
            if (controlX > 1){
                controlX = 1;
            } else if (controlX < 0){
                controlX = 0;
            }
            //Ensures R is within the range of -1 to 1
            double controlRX = -R;
            if (controlRX > 1){
                controlRX = 1;
            } else if (controlRX < 0){
                controlRX = 0;
            }
            //Ensures maxSpeed is within the range of -1 to 1
            if (maxSpeed > 1){
                maxSpeed = 1;
            } else if (maxSpeed < 0){
                maxSpeed = 0;
            }
            //Ensures T is not less than zero
            if (T < 0) {
                T = 0;
            }

            leftFront.setPower(((controlY - controlX) + controlRX) * maxSpeed);
            leftBack.setPower((controlY + controlX + controlRX) * maxSpeed);
            rightFront.setPower(((controlY - controlX) - controlRX) * maxSpeed);
            rightBack.setPower(((controlY + controlX) - controlRX) * maxSpeed);
            sleep(T);
            halt();
        }
        //DO NOT USE STRAFE UNLESS NEEDED
        public static void strafeLeft(double Speed, long time) {
            leftBack.setPower(Speed);
            leftFront.setPower(-Speed);
            rightBack.setPower(Speed);
            rightFront.setPower(-Speed);
            sleep(time);
            halt();
        }
        //DO NOT USE STRAFE UNLESS NEEDED
        public static void strafeRight(double Speed, long time) {
            leftBack.setPower(-Speed);
            leftFront.setPower(Speed);
            rightBack.setPower(-Speed);
            rightFront.setPower(Speed);
            sleep(time);
            halt();
        }

        public static void turnRight(double Speed, long time) {
            leftBack.setPower(Speed);
            leftFront.setPower(Speed);
            rightBack.setPower(Speed);
            rightFront.setPower(Speed);
            sleep(time);
            halt();
        }

        public static void turnLeft(double Speed, long time) {
            leftBack.setPower(-Speed);
            leftFront.setPower(-Speed);
            rightBack.setPower(-Speed);
            rightFront.setPower(-Speed);
            sleep(time);
            halt();
        }

        public static void moveBackward(double Speed, long time) {
            leftBack.setPower(-Speed);
            leftFront.setPower(-Speed);
            rightBack.setPower(Speed);
            rightFront.setPower(Speed);
            sleep(time);
            halt();
        }

        public static void moveForward(double Speed, long time) {
            leftBack.setPower(Speed);
            leftFront.setPower(Speed);
            rightBack.setPower(-Speed);
            rightFront.setPower(-Speed);
            sleep(time);
            halt();
        }

        public static void halt() {
            leftBack.setPower(0);
            leftFront.setPower(0);
            rightBack.setPower(0);
            rightFront.setPower(0);
        }
    }

    //Class for limelight functions
    public static class Limelight {
        public Limelight() {super();}

        //moves to a predetermined point on the field
        public static void localizer(double localPower, int sleepTime, boolean updatePose) {
            //loops until escaped
            while (true) {
                LLResult result = limelight.getLatestResult();
                //vertical min & max of the april tag
                double txMax = 14.700;
                double txMin = 14.400;
                /*
                not currently used, ty would be used for strafing
                double tyMax = 13.5;
                double tyMin = 12;
                */
                //min & max % of the camera that the april tag takes up
                double taMax = .88;
                double taMin = .91;
                //current values of tx, ty, ta
                double tx;
                double ty;
                double ta;
                //ensures non-null results
                if (result != null && result.isValid()) {
                    tx = result.getTx(); // How far left or right the target is (degrees)
                    ty = result.getTy(); // How far up or down the target is (degrees)
                    ta = result.getTa(); // How big the target looks (0%-100% of the image)
                    if (debug.debugTelemetry) {
                        telemetry.addData("Target X", tx);
                        telemetry.addData("Target Y", ty);
                        telemetry.addData("Target Area", ta);
                        telemetry.update();
                    }
                    if (tx < txMin) {
                        // turn right
                        motion.turnRight(localPower, sleepTime);
                    } else if (tx > txMax) {
                        //turn left
                        motion.turnLeft(localPower, sleepTime);
                    } else if (ta > taMax) {
                        //move backward
                        motion.moveBackward(localPower, sleepTime);
                    } else if (ta < taMin) {
                        //move forward
                        motion.moveForward(localPower, sleepTime);
                    } else {
                        break;
                    }
                    poseLimelight(updatePose);
                } else {
                    if (debug.debugTelemetry) {
                        telemetry.addData("Limelight", "No Targets");
                        telemetry.update();
                    }
                    //stop movement
                    motion.halt();
                }
            }
        }

        //Outputs a list of AprilTag IDs that the limelight can see
        public static int processLimeLightMotif() {
            //determines whether or not to run the function
            boolean processTrig = true;
            //array and individual ids
            int id;
            ArrayList<Integer> IDs = new ArrayList<>();
            int Motif;
            //current values of tx, ty, ta
            double tx;
            double ty;
            double ta;
            //processing trig allows loop to escape early or ends if the loop has exceeded 1000 iterations
            while (count <= 1000 && processTrig) {
                LLResult result = limelight.getLatestResult();
                //ensures non-null results
                if (result != null && result.isValid()) {
                    // Get the list of all AprilTags
                    List<LLResultTypes.FiducialResult> fiducialList = result.getFiducialResults();

                    tx = result.getTx(); // How far left or right the target is (degrees)
                    ty = result.getTy(); // How far up or down the target is (degrees)
                    ta = result.getTa(); // How big the target looks (0%-100% of the image)

                    if (debug.debugTelemetry) {
                        telemetry.addData("Target X", tx);
                        telemetry.addData("Target Y", ty);
                        telemetry.addData("Target Area", ta);
                        telemetry.update();
                    }
                    //checks if there are april tags
                    if (!fiducialList.isEmpty()) {
                        if (debug.debugTelemetry) {
                            telemetry.addData("Detections Found", fiducialList.size());
                            telemetry.update();
                        }
                        // Iterate through each detected tag
                        for (LLResultTypes.FiducialResult fiducial : fiducialList) {
                            id = fiducial.getFiducialId();
                            IDs.add(id);
                            if (debug.debugTelemetry) {
                                telemetry.addData("Tag ID", id);
                                telemetry.update();
                            }
                            //early escape
                            processTrig = false;
                        }
                    } else {
                        if (debug.debugTelemetry) {
                            telemetry.addData("Detections Found", "None");
                            telemetry.update();
                        }
                    }
                } else {
                    if (debug.debugTelemetry) {
                        telemetry.addData("Limelight Data", "Invalid or Stale");
                        assert result != null;
                        telemetry.addData("Staleness", result.getStaleness());
                        telemetry.update();
                    }
                }
                if (debug.debugTelemetry) {
                    telemetry.update();
                }
                //increments count
                count++;
                sleep(1);
            }
            //checks for motif ids
            if (IDs.contains(21)) {
                Motif = 1;
                telemetry.addData("Motif", "GPP " + Motif);
            } else if (IDs.contains(22)) {
                Motif = 2;
                telemetry.addData("Motif", "PGP " + Motif);
            } else if (IDs.contains(23)) {
                Motif = 3;
                telemetry.addData("Motif", "PPG " + Motif);
            } else {
                Motif = 0;
                telemetry.addData("Motif", "Check failed " + Motif);
            }
            telemetry.update();

            return Motif;
        }
        public static Pose2d poseLimelight(boolean updatePose) {
            LLResult result = limelight.getLatestResult();
            Pose3D pose;
            //ensures non-null results
            if (result != null && result.isValid()) {
                pose = result.getBotpose();
                //gets robot's 3D position and orientation
                telemetry.addData("X", (pose.getPosition().x * 39.37));
                telemetry.addData("Y",  (pose.getPosition().y * 39.37));
                telemetry.addData("Rotation",  pose.getOrientation().getYaw());
                telemetry.update();
                if (updatePose){
                    pinpoint.setPosition(conversions.pose.Pose3DToPose2D(pose));
                }
                //converts Pose3D to a Pose2d usable with RoadRunner
                return new Pose2d(pose.getPosition().x * 39.37, pose.getPosition().y * 39.37, pose.getOrientation().getYaw() - 180);
            } else {
                //failsafe if pose can't be obtained
                telemetry.addData("Limelight", "Failed to localize, defaulting to X:0 Y:0 θ:0");
                RobotLog.ii("Limelight", "Failed to localize, defaulting to X:0 Y:0 θ:0");
                RobotLog.addGlobalWarningMessage("Limelight", "Failed to localize, defaulting to X:0 Y:0 θ:0");
                if (updatePose) {
                    telemetry.addData("Pinpoint", "Pose Not Updated due to localizer failure");
                    RobotLog.ii("Pinpoint", "Pose Not Updated due to localizer failure");
                    RobotLog.addGlobalWarningMessage("Pinpoint", "Pose Not Updated due to localizer failure");
                }
                telemetry.update();

                return new Pose2d(0, 0, 0);
            }
        }
    }

    //Class for functions relating to the autonomous pathing tool RoadRunner
    public static class roadRunner {
        public roadRunner() {super();}

        //Creates a curved path for the robot to automatically follow
        public static class spline {
            public spline() {super();}
            //follows the arc of a circle when going to a 2d point
            public static Pose2d splineTo(double x, double y, double tangent, Pose2d startingPose) {

                MecanumDrive drive = new MecanumDrive(hwMap, startingPose);
                TrajectoryActionBuilder splineTo = drive.actionBuilder(startingPose)
                        .splineTo(new Vector2d(x, y), Math.toRadians(tangent));
                Actions.runBlocking(
                        new SequentialAction(
                                splineTo.build()));

                return (currentPose());
            }
            //follows the arc of a circle when going to a 2d point while facing a single angle
            public static Pose2d splineToConstantHeading(double x, double y, double tangent, Pose2d startingPose) {
                MecanumDrive drive = new MecanumDrive(hwMap, startingPose);
                TrajectoryActionBuilder splineToConstantHeading = drive.actionBuilder(startingPose)
                        .splineToConstantHeading(new Vector2d(x, y), Math.toRadians(tangent));
                Actions.runBlocking(
                        new SequentialAction(
                                splineToConstantHeading.build()));

                return (currentPose());
            }
            //follows the arc of a circle when going to a 2d point while allowing for an ending angle, similar to if splineTo and turnTo are chained
            public static Pose2d splineToLinearHeading(double x, double y, double angle, double tangent, Pose2d startingPose) {
                MecanumDrive drive = new MecanumDrive(hwMap, startingPose);
                TrajectoryActionBuilder splineToLinearHeading = drive.actionBuilder(startingPose)
                        .splineToLinearHeading(new Pose2d(new Vector2d(x, y), angle), tangent);
                Actions.runBlocking(
                        new SequentialAction(
                                splineToLinearHeading.build()));

                return currentPose();
            }
            //follows the arc of a circle when going to a 2d point while always facing a user determined angle
            public static Pose2d splineToSplineHeading(double x, double y, double angle, double tangent, Pose2d startingPose) {
                MecanumDrive drive = new MecanumDrive(hwMap, startingPose);
                TrajectoryActionBuilder splineToSplineHeading = drive.actionBuilder(startingPose)
                        .splineToSplineHeading(new Pose2d(new Vector2d(x, y), angle), tangent);
                Actions.runBlocking(
                        new SequentialAction(
                                splineToSplineHeading.build()));

                return (currentPose());
            }
        }

        //Creates a horizontal path for the robot to move along
        public static class strafe {
            public strafe() {super();}
            //strafes to a 2d point
            public static Pose2d strafeTo(double x, double y, boolean VelCon, boolean AccCon, Pose2d startingPose) {
                MecanumDrive drive = new MecanumDrive(hwMap, startingPose);
                TrajectoryActionBuilder strafeTo;
                if (VelCon && AccCon) {
                    strafeTo = drive.actionBuilder(startingPose)
                            .strafeTo(new Vector2d(x, y), drive.defaultVelConstraint, drive.defaultAccelConstraint);
                } else if (VelCon) {
                    strafeTo = drive.actionBuilder(startingPose)
                            .strafeTo(new Vector2d(x, y), drive.defaultVelConstraint, null);
                } else if (AccCon) {
                    strafeTo = drive.actionBuilder(startingPose)
                            .strafeTo(new Vector2d(x, y), null, drive.defaultAccelConstraint);
                } else {
                    strafeTo = drive.actionBuilder(startingPose)
                            .strafeTo(new Vector2d(x, y));
                }

                Actions.runBlocking(
                        new SequentialAction(
                                strafeTo.build()));

                return (currentPose());
            }
            //strafes to a 2d point while maintaining heading
            public static Pose2d strafeToConstantHeading(double x, double y, boolean VelCon, boolean AccCon, Pose2d startingPose) {
                MecanumDrive drive = new MecanumDrive(hwMap, startingPose);
                TrajectoryActionBuilder strafeToConstantHeading;
                if (VelCon && AccCon) {
                    strafeToConstantHeading = drive.actionBuilder(startingPose)
                            .strafeToConstantHeading(new Vector2d(x, y), drive.defaultVelConstraint, drive.defaultAccelConstraint);
                } else if (VelCon) {
                    strafeToConstantHeading = drive.actionBuilder(startingPose)
                            .strafeToConstantHeading(new Vector2d(x, y), drive.defaultVelConstraint, null);
                } else if (AccCon) {
                    strafeToConstantHeading = drive.actionBuilder(startingPose)
                            .strafeToConstantHeading(new Vector2d(x, y), null, drive.defaultAccelConstraint);
                } else {
                    strafeToConstantHeading = drive.actionBuilder(startingPose)
                            .strafeToConstantHeading(new Vector2d(x, y));
                }

                Actions.runBlocking(
                        new SequentialAction(
                                strafeToConstantHeading.build()));

                return (currentPose());
            }
            //strafes to a 2d point and ending at a user determined heading
            public static Pose2d strafeToLinearHeading(double x, double y, double angle, boolean VelCon, boolean AccCon, Pose2d startingPose) {
                MecanumDrive drive = new MecanumDrive(hwMap, startingPose);
                TrajectoryActionBuilder strafeToLinearHeading;
                if (VelCon && AccCon) {
                    strafeToLinearHeading = drive.actionBuilder(startingPose)
                            .strafeToLinearHeading(new Vector2d(x, y), Math.toRadians(angle), drive.defaultVelConstraint, drive.defaultAccelConstraint);
                } else if (VelCon) {
                    strafeToLinearHeading = drive.actionBuilder(startingPose)
                            .strafeToLinearHeading(new Vector2d(x, y), Math.toRadians(angle), drive.defaultVelConstraint, null);
                } else if (AccCon) {
                    strafeToLinearHeading = drive.actionBuilder(startingPose)
                            .strafeToLinearHeading(new Vector2d(x, y), Math.toRadians(angle), null, drive.defaultAccelConstraint);
                } else {
                    strafeToLinearHeading = drive.actionBuilder(startingPose)
                            .strafeToLinearHeading(new Vector2d(x, y), Math.toRadians(angle));
                }

                Actions.runBlocking(
                        new SequentialAction(
                                strafeToLinearHeading.build()));

                return (currentPose());
            }
            //strafes to a 2d point while facing a user determined heading
            public static Pose2d strafeToSplineHeading(double x, double y, double angle, boolean VelCon, boolean AccCon, Pose2d startingPose) {
                MecanumDrive drive = new MecanumDrive(hwMap, startingPose);
                TrajectoryActionBuilder strafeToSplineHeading;
                if (VelCon && AccCon) {
                    strafeToSplineHeading = drive.actionBuilder(startingPose)
                            .strafeToSplineHeading(new Vector2d(x, y), Math.toRadians(angle), drive.defaultVelConstraint, drive.defaultAccelConstraint);
                } else if (VelCon) {
                    strafeToSplineHeading = drive.actionBuilder(startingPose)
                            .strafeToSplineHeading(new Vector2d(x, y), Math.toRadians(angle), drive.defaultVelConstraint, null);
                } else if (AccCon) {
                    strafeToSplineHeading = drive.actionBuilder(startingPose)
                            .strafeToSplineHeading(new Vector2d(x, y), Math.toRadians(angle), null, drive.defaultAccelConstraint);
                } else {
                    strafeToSplineHeading = drive.actionBuilder(startingPose)
                            .strafeToSplineHeading(new Vector2d(x, y), Math.toRadians(angle));
                }

                Actions.runBlocking(
                        new SequentialAction(
                                strafeToSplineHeading.build()));

                return (currentPose());
            }
        }

        //takes a line to a point on a specified axis (x or y)
        public static class lineTo {
            public lineTo() {super();}
            //takes a linear path on the x-axis to a given x value
            public static Pose2d lineToX(double X, boolean VelCon, boolean AccCon, Pose2d startingPose) {
                MecanumDrive drive = new MecanumDrive(hwMap, startingPose);
                TrajectoryActionBuilder lineToX;
                if (VelCon && AccCon) {
                    lineToX = drive.actionBuilder(startingPose)
                            .lineToX(X, drive.defaultVelConstraint, drive.defaultAccelConstraint);
                } else if (VelCon) {
                    lineToX = drive.actionBuilder(startingPose)
                            .lineToX(X, drive.defaultVelConstraint, null);
                } else if (AccCon) {
                    lineToX = drive.actionBuilder(startingPose)
                            .lineToX(X, null, drive.defaultAccelConstraint);
                } else {
                    lineToX = drive.actionBuilder(startingPose)
                            .lineToX(X);
                }

                Actions.runBlocking(
                        new SequentialAction(
                                lineToX.build()));

                return (currentPose());
            }
            //takes a linear path on the x-axis to a given x value while maintaining its starting heading
            public static Pose2d lineToXConstantHeading(double X, boolean VelCon, boolean AccCon, Pose2d startingPose) {
                MecanumDrive drive = new MecanumDrive(hwMap, startingPose);
                TrajectoryActionBuilder lineToXConstantHeading;
                if (VelCon && AccCon) {
                    lineToXConstantHeading = drive.actionBuilder(startingPose)
                            .lineToXConstantHeading(X, drive.defaultVelConstraint, drive.defaultAccelConstraint);
                } else if (VelCon) {
                    lineToXConstantHeading = drive.actionBuilder(startingPose)
                            .lineToXConstantHeading(X, drive.defaultVelConstraint, null);
                } else if (AccCon) {
                    lineToXConstantHeading = drive.actionBuilder(startingPose)
                            .lineToXConstantHeading(X, null, drive.defaultAccelConstraint);
                } else {
                    lineToXConstantHeading = drive.actionBuilder(startingPose)
                            .lineToXConstantHeading(X);
                }

                Actions.runBlocking(
                        new SequentialAction(
                                lineToXConstantHeading.build()));

                return (currentPose());
            }
            //takes a linear path on the x-axis to a given x value and ending at a specified heading
            public static Pose2d lineToXLinearHeading(double X, double angle, boolean VelCon, boolean AccCon, Pose2d startingPose) {
                MecanumDrive drive = new MecanumDrive(hwMap, startingPose);
                TrajectoryActionBuilder lineToXLinearHeading;
                if (VelCon && AccCon) {
                    lineToXLinearHeading = drive.actionBuilder(startingPose)
                            .lineToXLinearHeading(X, Math.toRadians(angle), drive.defaultVelConstraint, drive.defaultAccelConstraint);
                } else if (VelCon) {
                    lineToXLinearHeading = drive.actionBuilder(startingPose)
                            .lineToXLinearHeading(X, Math.toRadians(angle), drive.defaultVelConstraint, null);
                } else if (AccCon) {
                    lineToXLinearHeading = drive.actionBuilder(startingPose)
                            .lineToXLinearHeading(X, Math.toRadians(angle), null, drive.defaultAccelConstraint);
                } else {
                    lineToXLinearHeading = drive.actionBuilder(startingPose)
                            .lineToXLinearHeading(X, Math.toRadians(angle));
                }

                Actions.runBlocking(
                        new SequentialAction(
                                lineToXLinearHeading.build()));

                return (currentPose());
            }
            //takes a linear path on the x-axis to a given x value while facing a user defined direction
            public static Pose2d lineToXSplineHeading(double X, double angle, boolean VelCon, boolean AccCon, Pose2d startingPose) {
                MecanumDrive drive = new MecanumDrive(hwMap, startingPose);
                TrajectoryActionBuilder lineToXSplineHeading;
                if (VelCon && AccCon) {
                    lineToXSplineHeading = drive.actionBuilder(startingPose)
                            .lineToXSplineHeading(X, Math.toRadians(angle), drive.defaultVelConstraint, drive.defaultAccelConstraint);
                } else if (VelCon) {
                    lineToXSplineHeading = drive.actionBuilder(startingPose)
                            .lineToXSplineHeading(X, Math.toRadians(angle), drive.defaultVelConstraint, null);
                } else if (AccCon) {
                    lineToXSplineHeading = drive.actionBuilder(startingPose)
                            .lineToXSplineHeading(X, Math.toRadians(angle), null, drive.defaultAccelConstraint);
                } else {
                    lineToXSplineHeading = drive.actionBuilder(startingPose)
                            .lineToXSplineHeading(X, Math.toRadians(angle));
                }

                Actions.runBlocking(
                        new SequentialAction(
                                lineToXSplineHeading.build()));

                return (currentPose());
            }
            //takes a linear path on the y-axis to a given y value
            public static Pose2d lineToY(double Y, boolean VelCon, boolean AccCon, Pose2d startingPose) {
                MecanumDrive drive = new MecanumDrive(hwMap, startingPose);
                TrajectoryActionBuilder lineToY;
                if (VelCon && AccCon) {
                    lineToY = drive.actionBuilder(startingPose)
                            .lineToY(Y, drive.defaultVelConstraint, drive.defaultAccelConstraint);
                } else if (VelCon) {
                    lineToY = drive.actionBuilder(startingPose)
                            .lineToY(Y, drive.defaultVelConstraint, null);
                } else if (AccCon) {
                    lineToY = drive.actionBuilder(startingPose)
                            .lineToY(Y, null, drive.defaultAccelConstraint);
                } else {
                    lineToY = drive.actionBuilder(startingPose)
                            .lineToY(Y);
                }

                Actions.runBlocking(
                        new SequentialAction(
                                lineToY.build()));

                return (currentPose());
            }
            //takes a linear path on the x-axis to a given x value while maintaining its starting heading
            public static Pose2d lineToYConstantHeading(double Y, boolean VelCon, boolean AccCon, Pose2d startingPose) {
                MecanumDrive drive = new MecanumDrive(hwMap, startingPose);
                TrajectoryActionBuilder lineToYConstantHeading;
                if (VelCon && AccCon) {
                    lineToYConstantHeading = drive.actionBuilder(startingPose)
                            .lineToYConstantHeading(Y, drive.defaultVelConstraint, drive.defaultAccelConstraint);
                } else if (VelCon) {
                    lineToYConstantHeading = drive.actionBuilder(startingPose)
                            .lineToYConstantHeading(Y, drive.defaultVelConstraint, null);
                } else if (AccCon) {
                    lineToYConstantHeading = drive.actionBuilder(startingPose)
                            .lineToYConstantHeading(Y, null, drive.defaultAccelConstraint);
                } else {
                    lineToYConstantHeading = drive.actionBuilder(startingPose)
                            .lineToYConstantHeading(Y);
                }

                Actions.runBlocking(
                        new SequentialAction(
                                lineToYConstantHeading.build()));

                return (currentPose());
            }
            //takes a linear path on the x-axis to a given x value and ending at a specified heading
            public static Pose2d lineToYLinearHeading(double Y, double angle, boolean VelCon, boolean AccCon, Pose2d startingPose) {
                MecanumDrive drive = new MecanumDrive(hwMap, startingPose);
                TrajectoryActionBuilder lineToYLinearHeading;
                if (VelCon && AccCon) {
                    lineToYLinearHeading = drive.actionBuilder(startingPose)
                            .lineToYLinearHeading(Y, Math.toRadians(angle), drive.defaultVelConstraint, drive.defaultAccelConstraint);
                } else if (VelCon) {
                    lineToYLinearHeading = drive.actionBuilder(startingPose)
                            .lineToYLinearHeading(Y, Math.toRadians(angle), drive.defaultVelConstraint, null);
                } else if (AccCon) {
                    lineToYLinearHeading = drive.actionBuilder(startingPose)
                            .lineToYLinearHeading(Y, Math.toRadians(angle), null, drive.defaultAccelConstraint);
                } else {
                    lineToYLinearHeading = drive.actionBuilder(startingPose)
                            .lineToYLinearHeading(Y, Math.toRadians(angle));
                }

                Actions.runBlocking(
                        new SequentialAction(
                                lineToYLinearHeading.build()));

                return (currentPose());
            }
            //takes a linear path on the x-axis to a given x value while facing a user defined direction
            public static Pose2d lineToYSplineHeading(double Y, double angle, boolean VelCon, boolean AccCon, Pose2d startingPose) {
                MecanumDrive drive = new MecanumDrive(hwMap, startingPose);
                TrajectoryActionBuilder lineToYSplineHeading;
                if (VelCon && AccCon) {
                    lineToYSplineHeading = drive.actionBuilder(startingPose)
                            .lineToYSplineHeading(Y, Math.toRadians(angle), drive.defaultVelConstraint, drive.defaultAccelConstraint);
                } else if (VelCon) {
                    lineToYSplineHeading = drive.actionBuilder(startingPose)
                            .lineToYSplineHeading(Y, Math.toRadians(angle), drive.defaultVelConstraint, null);
                } else if (AccCon) {
                    lineToYSplineHeading = drive.actionBuilder(startingPose)
                            .lineToYSplineHeading(Y, Math.toRadians(angle), null, drive.defaultAccelConstraint);
                } else {
                    lineToYSplineHeading = drive.actionBuilder(startingPose)
                            .lineToYSplineHeading(Y, Math.toRadians(angle));
                }

                Actions.runBlocking(
                        new SequentialAction(
                                lineToYSplineHeading.build()));

                return (currentPose());
            }
        }

        //turns to a specified angle
        public static Pose2d turnTo(double angle, Pose2d startingPose) {
            MecanumDrive drive = new MecanumDrive(hwMap, startingPose);
            TrajectoryActionBuilder turnTo = drive.actionBuilder(startingPose)
                    .turnTo(angle);
            Actions.runBlocking(
                    new SequentialAction(
                            turnTo.build()));

            return (currentPose());

        }

        //turns to a specified angle
        public static Pose2d turn(double angle, Pose2d startingPose) {
            MecanumDrive drive = new MecanumDrive(hwMap, startingPose);
            TrajectoryActionBuilder turn = drive.actionBuilder(startingPose)
                    .turn(angle);
            Actions.runBlocking(
                    new SequentialAction(
                            turn.build()));

            return (currentPose());
        }
    }

    //Class for non-critical utility systems
    public static class utils {
        public utils() {super();}
        //Class for managing telemetry systems
        public static class telemetrySys {
            telemetrySys() {super();}
            //outputs motor telemetry
            public static class motor {
                motor() {super();}
                public static void leftBackMotor() {
                    telemetry.addData("Left Back", leftBack);
                    telemetry.addData("Left Back", leftBack.getConnectionInfo());
                    telemetry.addData("Left Back", leftBack.getDeviceName());
                    telemetry.addData("Left Back", leftBack.getDirection());
                    telemetry.addData("Left Back", leftBack.getManufacturer());
                    telemetry.addData("Left Back", leftBack.getVersion());
                    telemetry.addData("Left Back", leftBack.getClass());
                    telemetry.addData("Left Back", leftBack.isBusy());
                    telemetry.addData("Left Back", leftBack.getController());
                    telemetry.addData("Left Back", leftBack.getMotorType());
                    telemetry.addData("Left Back", leftBack.getCurrentPosition());
                    telemetry.addData("Left Back", leftBack.getMode());
                    telemetry.addData("Left Back", leftBack.getPortNumber());
                    telemetry.addData("Left Back", leftBack.getPowerFloat());
                    telemetry.addData("Left Back", leftBack.getTargetPosition());
                    telemetry.addData("Left Back", leftBack.getZeroPowerBehavior());
                    telemetry.update();
                }
                public static void rightBackMotor() {
                    telemetry.addData("Right Back", rightBack);
                    telemetry.addData("Right Back", rightBack.getConnectionInfo());
                    telemetry.addData("Right Back", rightBack.getDeviceName());
                    telemetry.addData("Right Back", rightBack.getDirection());
                    telemetry.addData("Right Back", rightBack.getManufacturer());
                    telemetry.addData("Right Back", rightBack.getVersion());
                    telemetry.addData("Right Back", rightBack.getClass());
                    telemetry.addData("Right Back", rightBack.isBusy());
                    telemetry.addData("Right Back", rightBack.getController());
                    telemetry.addData("Right Back", rightBack.getMotorType());
                    telemetry.addData("Right Back", rightBack.getCurrentPosition());
                    telemetry.addData("Right Back", rightBack.getMode());
                    telemetry.addData("Right Back", rightBack.getPortNumber());
                    telemetry.addData("Right Back", rightBack.getPowerFloat());
                    telemetry.addData("Right Back", rightBack.getTargetPosition());
                    telemetry.addData("Right Back", rightBack.getZeroPowerBehavior());
                    telemetry.update();
                }
                public static void rightFrontMotor() {
                    telemetry.addData("Right Front", rightFront);
                    telemetry.addData("Right Front", rightFront.getConnectionInfo());
                    telemetry.addData("Right Front", rightFront.getDeviceName());
                    telemetry.addData("Right Front", rightFront.getDirection());
                    telemetry.addData("Right Front", rightFront.getManufacturer());
                    telemetry.addData("Right Front", rightFront.getVersion());
                    telemetry.addData("Right Front", rightFront.getClass());
                    telemetry.addData("Right Front", rightFront.isBusy());
                    telemetry.addData("Right Front", rightFront.getController());
                    telemetry.addData("Right Front", rightFront.getMotorType());
                    telemetry.addData("Right Front", rightFront.getCurrentPosition());
                    telemetry.addData("Right Front", rightFront.getMode());
                    telemetry.addData("Right Front", rightFront.getPortNumber());
                    telemetry.addData("Right Front", rightFront.getPowerFloat());
                    telemetry.addData("Right Front", rightFront.getTargetPosition());
                    telemetry.addData("Right Front", rightFront.getZeroPowerBehavior());
                    telemetry.update();
                }
                public static void leftFrontMotor() {
                    telemetry.addData("Left Front", leftFront);
                    telemetry.addData("Left Front", leftFront.getConnectionInfo());
                    telemetry.addData("Left Front", leftFront.getDeviceName());
                    telemetry.addData("Left Front", leftFront.getDirection());
                    telemetry.addData("Left Front", leftFront.getManufacturer());
                    telemetry.addData("Left Front", leftFront.getVersion());
                    telemetry.addData("Left Front", leftFront.getClass());
                    telemetry.addData("Left Front", leftFront.isBusy());
                    telemetry.addData("Left Front", leftFront.getController());
                    telemetry.addData("Left Front", leftFront.getMotorType());
                    telemetry.addData("Left Front", leftFront.getCurrentPosition());
                    telemetry.addData("Left Front", leftFront.getMode());
                    telemetry.addData("Left Front", leftFront.getPortNumber());
                    telemetry.addData("Left Front", leftFront.getPowerFloat());
                    telemetry.addData("Left Front", leftFront.getTargetPosition());
                    telemetry.addData("Left Front", leftFront.getZeroPowerBehavior());
                    telemetry.update();
                }
                public static void leftLauncherMotor() {
                    telemetry.addData("Left Launcher", leftLauncher);
                    telemetry.addData("Left Launcher", leftLauncher.getConnectionInfo());
                    telemetry.addData("Left Launcher", leftLauncher.getDeviceName());
                    telemetry.addData("Left Launcher", leftLauncher.getDirection());
                    telemetry.addData("Left Launcher", leftLauncher.getManufacturer());
                    telemetry.addData("Left Launcher", leftLauncher.getVersion());
                    telemetry.addData("Left Launcher", leftLauncher.getClass());
                    telemetry.addData("Left Launcher", leftLauncher.isBusy());
                    telemetry.addData("Left Launcher", leftLauncher.getController());
                    telemetry.addData("Left Launcher", leftLauncher.getMotorType());
                    telemetry.addData("Left Launcher", leftLauncher.getCurrentPosition());
                    telemetry.addData("Left Launcher", leftLauncher.getMode());
                    telemetry.addData("Left Launcher", leftLauncher.getPortNumber());
                    telemetry.addData("Left Launcher", leftLauncher.getPowerFloat());
                    telemetry.addData("Left Launcher", leftLauncher.getTargetPosition());
                    telemetry.addData("Left Launcher", leftLauncher.getZeroPowerBehavior());
                    telemetry.update();
                }
                public static void rightLauncherMotor() {
                    telemetry.addData("Right Launcher", rightLauncher);
                    telemetry.addData("Right Launcher", rightLauncher.getConnectionInfo());
                    telemetry.addData("Right Launcher", rightLauncher.getDeviceName());
                    telemetry.addData("Right Launcher", rightLauncher.getDirection());
                    telemetry.addData("Right Launcher", rightLauncher.getManufacturer());
                    telemetry.addData("Right Launcher", rightLauncher.getVersion());
                    telemetry.addData("Right Launcher", rightLauncher.getClass());
                    telemetry.addData("Right Launcher", rightLauncher.isBusy());
                    telemetry.addData("Right Launcher", rightLauncher.getController());
                    telemetry.addData("Right Launcher", rightLauncher.getMotorType());
                    telemetry.addData("Right Launcher", rightLauncher.getCurrentPosition());
                    telemetry.addData("Right Launcher", rightLauncher.getMode());
                    telemetry.addData("Right Launcher", rightLauncher.getPortNumber());
                    telemetry.addData("Right Launcher", rightLauncher.getPowerFloat());
                    telemetry.addData("Right Launcher", rightLauncher.getTargetPosition());
                    telemetry.addData("Right Launcher", rightLauncher.getZeroPowerBehavior());
                    telemetry.update();
                }
                public static void liftMotor() {
                    telemetry.addData("Lift", lift);
                    telemetry.addData("Lift", lift.getConnectionInfo());
                    telemetry.addData("Lift", lift.getDeviceName());
                    telemetry.addData("Lift", lift.getDirection());
                    telemetry.addData("Lift", lift.getManufacturer());
                    telemetry.addData("Lift", lift.getVersion());
                    telemetry.addData("Lift", lift.getClass());
                    telemetry.addData("Lift", lift.isBusy());
                    telemetry.addData("Lift", lift.getController());
                    telemetry.addData("Lift", lift.getMotorType());
                    telemetry.addData("Lift", lift.getCurrentPosition());
                    telemetry.addData("Lift", lift.getMode());
                    telemetry.addData("Lift", lift.getPortNumber());
                    telemetry.addData("Lift", lift.getPowerFloat());
                    telemetry.addData("Lift", lift.getTargetPosition());
                    telemetry.addData("Lift", lift.getZeroPowerBehavior());
                    telemetry.update();
                }
                public static void intakeMotor() {
                    telemetry.addData("Intake", intakeMotor);
                    telemetry.addData("Intake", intakeMotor.getConnectionInfo());
                    telemetry.addData("Intake", intakeMotor.getDeviceName());
                    telemetry.addData("Intake", intakeMotor.getDirection());
                    telemetry.addData("Intake", intakeMotor.getManufacturer());
                    telemetry.addData("Intake", intakeMotor.getVersion());
                    telemetry.addData("Intake", intakeMotor.getClass());
                    telemetry.addData("Intake", intakeMotor.isBusy());
                    telemetry.addData("Intake", intakeMotor.getController());
                    telemetry.addData("Intake", intakeMotor.getMotorType());
                    telemetry.addData("Intake", intakeMotor.getCurrentPosition());
                    telemetry.addData("Intake", intakeMotor.getMode());
                    telemetry.addData("Intake", intakeMotor.getPortNumber());
                    telemetry.addData("Intake", intakeMotor.getPowerFloat());
                    telemetry.addData("Intake", intakeMotor.getTargetPosition());
                    telemetry.addData("Intake", intakeMotor.getZeroPowerBehavior());
                    telemetry.update();
                }
            }
            //outputs servo telemetry
            public static class servos {
                servos() {super();}
                public static void turnTable() {
                    telemetry.addData("Turn Table", turnTableServo);
                    telemetry.addData("Turn Table", turnTableServo.getController());
                    telemetry.addData("Turn Table", turnTableServo.getDirection());
                    telemetry.addData("Turn Table", turnTableServo.getPortNumber());
                    telemetry.addData("Turn Table", turnTableServo.getPosition());
                    telemetry.addData("Turn Table", turnTableServo.getConnectionInfo());
                    telemetry.addData("Turn Table", turnTableServo.getDeviceName());
                    telemetry.addData("Turn Table", turnTableServo.getManufacturer());
                    telemetry.addData("Turn Table", turnTableServo.getVersion());
                    telemetry.addData("Turn Table", turnTableServo.getClass());
                    telemetry.update();
                }
                public static void intakeDoor() {
                    telemetry.addData("Front Door", frontDoor);
                    telemetry.addData("Front Door", frontDoor.getController());
                    telemetry.addData("Front Door", frontDoor.getDirection());
                    telemetry.addData("Front Door", frontDoor.getPortNumber());
                    telemetry.addData("Front Door", frontDoor.getPosition());
                    telemetry.addData("Front Door", frontDoor.getConnectionInfo());
                    telemetry.addData("Front Door", frontDoor.getDeviceName());
                    telemetry.addData("Front Door", frontDoor.getManufacturer());
                    telemetry.addData("Front Door", frontDoor.getVersion());
                    telemetry.addData("Front Door", frontDoor.getClass());
                    telemetry.update();
                }
                public static void launchDoor() {
                    telemetry.addData("Back Door", backDoor);
                    telemetry.addData("Back Door", backDoor.getController());
                    telemetry.addData("Back Door", backDoor.getDirection());
                    telemetry.addData("Back Door", backDoor.getPortNumber());
                    telemetry.addData("Back Door", backDoor.getPosition());
                    telemetry.addData("Back Door", backDoor.getConnectionInfo());
                    telemetry.addData("Back Door", backDoor.getDeviceName());
                    telemetry.addData("Back Door", backDoor.getManufacturer());
                    telemetry.addData("Back Door", backDoor.getVersion());
                    telemetry.addData("Back Door", backDoor.getClass());
                    telemetry.update();
                }
                public static void launchScoop() {
                    telemetry.addData("Scoop", scoop);
                    telemetry.addData("Scoop", scoop.getController());
                    telemetry.addData("Scoop", scoop.getDirection());
                    telemetry.addData("Scoop", scoop.getPortNumber());
                    telemetry.addData("Scoop", scoop.getPosition());
                    telemetry.addData("Scoop", scoop.getConnectionInfo());
                    telemetry.addData("Scoop", scoop.getDeviceName());
                    telemetry.addData("Scoop", scoop.getManufacturer());
                    telemetry.addData("Scoop", scoop.getVersion());
                    telemetry.addData("Scoop", scoop.getClass());
                    telemetry.update();
                }
                public static void leftLaunchLift() {
                    telemetry.addData("Left Launch Lift", launchLiftLeft);
                    telemetry.addData("Left Launch Lift", launchLiftLeft.getController());
                    telemetry.addData("Left Launch Lift", launchLiftLeft.getDirection());
                    telemetry.addData("Left Launch Lift", launchLiftLeft.getPortNumber());
                    telemetry.addData("Left Launch Lift", launchLiftLeft.getPower());
                    telemetry.addData("Left Launch Lift", launchLiftLeft.getConnectionInfo());
                    telemetry.addData("Left Launch Lift", launchLiftLeft.getDeviceName());
                    telemetry.addData("Left Launch Lift", launchLiftLeft.getManufacturer());
                    telemetry.addData("Left Launch Lift", launchLiftLeft.getVersion());
                    telemetry.addData("Left Launch Lift", launchLiftLeft.getClass());
                    telemetry.update();
                }
                public static void rightLaunchLift() {
                    telemetry.addData("Right Launch Lift", launchLiftRight);
                    telemetry.addData("Right Launch Lift", launchLiftRight.getController());
                    telemetry.addData("Right Launch Lift", launchLiftRight.getDirection());
                    telemetry.addData("Right Launch Lift", launchLiftRight.getPortNumber());
                    telemetry.addData("Right Launch Lift", launchLiftRight.getPower());
                    telemetry.addData("Right Launch Lift", launchLiftRight.getConnectionInfo());
                    telemetry.addData("Right Launch Lift", launchLiftRight.getDeviceName());
                    telemetry.addData("Right Launch Lift", launchLiftRight.getManufacturer());
                    telemetry.addData("Right Launch Lift", launchLiftRight.getVersion());
                    telemetry.addData("Right Launch Lift", launchLiftRight.getClass());
                    telemetry.update();
                }
            }
        }
        //Function to manage the color of LED(s) on the robot
        public static void ledManager(String type, int ledNumber) {
            CRServo led = leds.get(ledNumber);
            if (led != null) {
                switch (type) {
                    case "Clear":
                        led.setPower(.5); //White

                        break;
                    case "Good":
                        led.setPower(0); //Green

                        break;
                    case "Warn":
                        led.setPower(-.25); //Yellow

                        break;
                    case "Alert":
                        led.setPower(-.35); //Orange

                        break;
                    case "Error":
                        led.setPower(-0.44); //red

                        break;
                    case "Null":
                        led.setPower(-.6); //Blank

                        break;
                    case "Match Alert":
                        led.setPower(0); //Purple

                        break;
                    case "Blue":
                        led.setPower(0.216); //Blue

                        break;
                    case "Purple":
                        led.setPower(0.415); //Purple

                        break;
                    case "Pink":
                        led.setPower(0.275); //Pink

                        break;
                    default:
                        if (debug.debugTelemetry) {
                            telemetry.addData("Led Manager Error", "Wrong or Invalid Input");
                            RobotLog.ii("Led Manager Error", "Wrong or Invalid Input");
                        }
                        break;
                }
            }
        }

        public static class auto {
            public auto() {super();}
            //automatically intakes based on if the launcher is up or down
            public void intake3Balls(double searchSpeed, double returnSpeed, double returnDistance, int kickTime) {
                int safeTrig;
                turnTableServo.setPosition(0);
                frontDoor.setPosition(1);
                intakeMotor.setPower(0.8);
                safeTrig = BackwardsTillBump(searchSpeed, 0);
                if (safeTrig == 1) {
                    moveForwardTics(returnSpeed, returnDistance * ticPerIn);
                    halfKick(kickTime);
                    sleep(250);
                    turnTableServo.setPosition(0.5);
                    frontDoor.setPosition(1);

                    safeTrig = BackwardsTillBump(searchSpeed, 0);
                    if (safeTrig == 1) {
                        moveForwardTics(returnSpeed, returnDistance * ticPerIn);
                        halfKick(kickTime);
                        sleep(250);
                        turnTableServo.setPosition(1);
                        frontDoor.setPosition(1);
                        safeTrig = BackwardsTillBump(searchSpeed, 0);
                        if (safeTrig == 1) {
                            halfKick(kickTime);
                            intakeMotor.setPower(0);

                        } else {

                            frontDoor.setPosition(.5);
                            intakeMotor.setPower(0);
                        }

                    } else {
                        frontDoor.setPosition(.5);
                        intakeMotor.setPower(0);
                    }
                } else {
                    frontDoor.setPosition(.5);
                    intakeMotor.setPower(0);
                }
            }
            //moves backwards for a certain of tics
            private void moveForwardTics(double Speed, double tic) {
                pinpoint.update();
                double xvalue = pinpoint.getEncoderX();
                while (xvalue - pinpoint.getEncoderX() <= tic) {
                    leftBack.setPower(-Speed);
                    leftFront.setPower(-Speed);
                    rightBack.setPower(-Speed);
                    rightFront.setPower(-Speed);
                }
                motion.halt();
            }
            //moves backwards until a sensor is triggered
            public int BackwardsTillBump(double Speed, int delay) {
                int returnSave;

                ElapsedTime BackwardsTillBumpClock = new ElapsedTime();

                BackwardsTillBumpClock.reset();
                while (BackwardsTillBumpClock.seconds() <= 2 && !(!intakeBump1.isPressed() || intakeBump2.isPressed())) {
                    leftBack.setPower(Speed);
                    leftFront.setPower(Speed);
                    rightBack.setPower(Speed);
                    rightFront.setPower(Speed);
                    sleep(1);
                    count++;
                }
                count = 0;

                if (!intakeBump1.isPressed() || intakeBump2.isPressed()) {
                    returnSave = 1;
                } else if (BackwardsTillBumpClock.seconds() >= 2) {
                    returnSave = 0;
                } else {
                    returnSave = 1;
                }

                leftBack.setPower(0);
                leftFront.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);
                return returnSave;
            }
            //moves kicker inside before returning it to door position
            public void halfKick(int time) {
                frontDoor.setPosition(0);
                sleep(time);
                frontDoor.setPosition(.5);
            }
        }
    }
}