package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.RobotLog;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.TouchSensor;

import java.util.ArrayList;
import java.util.Arrays;

public class HWI { //HWI = Hardware Interface
	private static OpMode OpMode;
	private LinearOpMode Linear;
	private static boolean debug;

	private DcMotorEx leftBack, rightBack, leftFront, rightFront, leftLauncher, rightLauncher, intakeMotor, lift;
	private CRServo launchLiftRight, launchLiftLeft;
	private Servo scoop, turnTableServo, backDoor, kicker;
	private GoBildaPinpointDriver pinpoint;
	private Limelight3A limelight;
	@SuppressWarnings("FieldCanBeLocal")
	private TouchSensor topBump, bottomBump, intakeBump1, intakeBump2;
	private ArrayList<CRServo> LEDs;
	private CRServo LED1;

	//HWI name = new HWI(this);
	public HWI(OpMode This) {
		if (This != null) {
			OpMode = This;
		} else {
			RobotLog.ii("HWI", "OpMode not provided, please check HWI's initialization statement.");
		}
	}
	
	public void DebugMode(boolean isEnabled) {
		debug = isEnabled;
	}

	public void init() {

		//Drive Definitions
		leftBack = OpMode.hardwareMap.get(DcMotorEx.class, "leftBack");
		rightBack = OpMode.hardwareMap.get(DcMotorEx.class, "rightBack");
		leftFront = OpMode.hardwareMap.get(DcMotorEx.class, "leftFront");
		rightFront = OpMode.hardwareMap.get(DcMotorEx.class, "rightFront");
		//Intake Definitions
		intakeMotor = OpMode.hardwareMap.get(DcMotorEx.class, "intakeMotor");
		kicker = OpMode.hardwareMap.get(Servo.class, "goofyAhhhhFrontDoor");
		intakeBump1 = OpMode.hardwareMap.get(TouchSensor.class, "intakeBump1");
		intakeBump2 = OpMode.hardwareMap.get(TouchSensor.class, "intakeBump2");
		//Launcher Definitions
		leftLauncher = OpMode.hardwareMap.get(DcMotorEx.class, "leftLauncher");
		rightLauncher = OpMode.hardwareMap.get(DcMotorEx.class, "rightLauncher");
		launchLiftRight = OpMode.hardwareMap.get(CRServo.class, "launchLiftRight");
		launchLiftLeft = OpMode.hardwareMap.get(CRServo.class, "launchLiftLeft");
		topBump = OpMode.hardwareMap.get(TouchSensor.class, "topBump");
		bottomBump = OpMode.hardwareMap.get(TouchSensor.class, "bottomBump");
		backDoor = OpMode.hardwareMap.get(Servo.class, "backDoor");
		scoop = OpMode.hardwareMap.get(Servo.class, "scoop");
		//Lift/Skis Definition
		lift = OpMode.hardwareMap.get(DcMotorEx.class, "lift");
		//Turntable Definition
		turnTableServo = OpMode.hardwareMap.get(Servo.class, "turnTableServo");
		//Pinpoint Definition
		pinpoint = OpMode.hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
		//LED Definition
		LED1 = OpMode.hardwareMap.get(CRServo.class, "Led1");
		//Limelight Definition
		limelight = OpMode.hardwareMap.get(Limelight3A.class, "limelight");
		//Drive Config
		leftBack.setDirection(DcMotor.Direction.FORWARD);
		rightBack.setDirection(DcMotor.Direction.FORWARD);
		leftFront.setDirection(DcMotor.Direction.REVERSE);
		rightFront.setDirection(DcMotor.Direction.FORWARD);
		rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
		pinpoint.initialize(); //Initializes odometry for use in code
		pinpoint.update();
		//LED Config
		LEDs = new ArrayList<>(Arrays.asList(null, LED1)); //creates a list of LEDs for ledManager to use
		//Limelight Config/Setup
		limelight.pipelineSwitch(0); //Sets the config the limelight should use
		limelight.setPollRateHz(100); //Limelight data polling rate
		limelight.start(); //Initializes limelight for use in code
	}

	public void drive(double linear, double lateral, double rotational, double speed) {
		linear = Utils.clamp(linear, -1, 1);
		lateral = Utils.clamp(lateral, -1, 1);
		rotational = Utils.clamp(rotational, -1, 1);
		speed = Utils.clamp(speed, -1, 1);
		leftFront.setPower(((linear + lateral) - rotational) * speed);
		leftBack.setPower((linear - lateral - rotational) * speed);
		rightFront.setPower(((linear + lateral) + rotational) * speed);
		rightBack.setPower(((linear - lateral) + rotational) * speed);
	}

	public void velocityDrive(double linear, double lateral, double rotational, double RPM , int ticksPerRev, double gearRatio) {
		Utils.ifLog(debug, "LIN + LAT + ROT + RPM + TPR + GR", String.valueOf(linear) + " " + String.valueOf(lateral) + " " + String.valueOf(rotational) + " " + String.valueOf(RPM) + " " + String.valueOf(ticksPerRev) + " " + String.valueOf(gearRatio));
		linear = Utils.clamp(linear, -1, 1);
		lateral = Utils.clamp(lateral, -1, 1);
		rotational = Utils.clamp(rotational, -1, 1);
		double TPS = ((RPM / 60) * ticksPerRev) / gearRatio;
		leftFront.setVelocity(((linear + lateral) - rotational) * TPS);
		leftBack.setVelocity((linear - lateral - rotational) * TPS);
		rightFront.setVelocity(((linear + lateral) + rotational) * TPS);
		rightBack.setVelocity(((linear - lateral) + rotational) * TPS);
	}

	public void launcherSpeed(double power) {
		Utils.ifLog(debug, "LaunchPwrPC", String.valueOf(power));
		power = Utils.clamp(power, -1, 1);
		leftLauncher.setPower(power);
		rightLauncher.setPower(power);
	}

	public void launcherVelocity(double RPM, double gearRatio, int ticksPerRev) {
		//Clamps lower bound of gearRatio
		if (gearRatio <= 0) {gearRatio = 1;}
		Utils.ifLog(debug, "VelLaunchRPM", String.valueOf(RPM));
		//converts RPM to the ticks per second required by setVelocity
		double TPS = ((RPM / 60) * ticksPerRev) / gearRatio;
		Utils.ifLog(debug, "VelLaunchTPS", String.valueOf(TPS));
		leftLauncher.setVelocity(TPS);
		rightLauncher.setVelocity(TPS);
	}

	private static class Utils {
		private static double clamp(double value, double min, double max) {
			ifLog(debug, "Utils.clamp",  "Clamping " + String.valueOf(value) + "between " + String.valueOf(min) + " " + String.valueOf(max));
			if (value < min) {
				ifLog(debug, "Utils.clamp",  String.valueOf(value) + " clamped at lower bound to " + String.valueOf(min));
				return min;
			} else if (value > max) {
				ifLog(debug, "Utils.clamp",  String.valueOf(value) + " clamped at upper bound to " + String.valueOf(max));
				return max;
			} else
				ifLog(debug, "Utils.clamp",  String.valueOf(value) + " fell within expected bounds");
				return value;
		}
		private static void sleep(long milliseconds) {
			try {
				ifLog(debug, "Utils.sleep",  "Attempting to sleep for " + String.valueOf(milliseconds/1000) + " seconds");
				Thread.sleep(milliseconds);
			} catch (InterruptedException e) {
				Thread.currentThread().interrupt();
			}
		}
		private static void ifLog(boolean IF, String caption, String data){
			if (IF) {
				RobotLog.dd(caption, data);
			}
		}
	}
}
