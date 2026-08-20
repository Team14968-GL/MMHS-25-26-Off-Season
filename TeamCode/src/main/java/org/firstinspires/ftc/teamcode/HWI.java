package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
	private OpMode OpMode;
	private LinearOpMode Linear;

	private DcMotor leftBack, rightBack, leftFront, rightFront, leftLauncher, rightLauncher, intakeMotor, lift;
	private CRServo launchLiftRight, launchLiftLeft;
	private Servo scoop, turnTableServo, backDoor, frontDoor;
	private GoBildaPinpointDriver pinpoint;
	private Limelight3A limelight;
	@SuppressWarnings("FieldCanBeLocal")
	private TouchSensor topBump, bottomBump, intakeBump1, intakeBump2;
	private ArrayList<CRServo> leds;
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
		debug = Enabled
	}

	public void init() {

		//Drive Definitions
		leftBack = OpMode.hardwareMap.get(DcMotor.class, "leftBack");
		rightBack = OpMode.hardwareMap.get(DcMotor.class, "rightBack");
		leftFront = OpMode.hardwareMap.get(DcMotor.class, "leftFront");
		rightFront = OpMode.hardwareMap.get(DcMotor.class, "rightFront");
		//Intake Definitions
		intakeMotor = OpMode.hardwareMap.get(DcMotor.class, "intakeMotor");
		frontDoor = OpMode.hardwareMap.get(Servo.class, "goofyAhhhhFrontDoor");
		intakeBump1 = OpMode.hardwareMap.get(TouchSensor.class, "intakeBump1");
		intakeBump2 = OpMode.hardwareMap.get(TouchSensor.class, "intakeBump2");
		//Launcher Definitions
		leftLauncher = OpMode.hardwareMap.get(DcMotor.class, "leftLauncher");
		rightLauncher = OpMode.hardwareMap.get(DcMotor.class, "rightLauncher");
		launchLiftRight = OpMode.hardwareMap.get(CRServo.class, "launchLiftRight");
		launchLiftLeft = OpMode.hardwareMap.get(CRServo.class, "launchLiftLeft");
		topBump = OpMode.hardwareMap.get(TouchSensor.class, "TopBump");
		bottomBump = OpMode.hardwareMap.get(TouchSensor.class, "BottomBump");
		backDoor = OpMode.hardwareMap.get(Servo.class, "backDoor");
		scoop = OpMode.hardwareMap.get(Servo.class, "scoop");
		//Lift/Skis Definition
		lift = OpMode.hardwareMap.get(DcMotor.class, "lift");
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
		pinpoint.initialize(); //Initializes odometry for use in code
		pinpoint.update();
		//LED Config
		leds = new ArrayList<>(Arrays.asList(null, LED1)); //creates a list of LEDs for ledManager to use
		//Limelight Config/Setup
		limelight.pipelineSwitch(0); //Sets the config the limelight should use
		limelight.setPollRateHz(100); //Limelight data polling rate
		limelight.start(); //Initializes limelight for use in code
	}

	public void drive(double linear, double lateral, double rotational, double speed) {
		linear = Utils.clamp(linear, 0, 1);
		lateral = Utils.clamp(lateral, 0, 1);
		rotational = Utils.clamp(rotational, 0, 1);
		speed = Utils.clamp(speed, 0, 1);
		leftFront.setPower(((linear + lateral) - rotational) * speed);
		leftBack.setPower((linear - lateral - rotational) * speed);
		rightFront.setPower(((linear + lateral) + rotational) * speed);
		rightBack.setPower(((linear - lateral) + rotational) * speed);
	}

	public void launcherSpeed(double power) {
		leftLauncher.setPower(power);
		rightLauncher.setPower(power);
	}

	private static class Utils {
		private static double clamp(double value, double min, double max) {
			if (value < min) {
				return min;
			} else if (value > max) {
				return max;
			} else return value;
		}
		private static void sleep(long milliseconds) {
			try {
				Thread.sleep(milliseconds);
			} catch (InterruptedException e) {
				Thread.currentThread().interrupt();
			}
		}
	}
}
