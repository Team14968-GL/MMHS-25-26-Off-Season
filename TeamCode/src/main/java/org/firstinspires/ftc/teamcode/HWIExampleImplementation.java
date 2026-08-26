package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@SuppressWarnings("unused")
@TeleOp
public class HWIExampleImplementation extends OpMode {
	// Below the Hardware Interface is initialize by passing the opmode to it
	final HWI robot = new HWI(this, null);

	@Override
	public void init() {
		// robot.init(); runs all of the code needed to start up the robot
		robot.init();
		// The debugMode flag outputs what HWI is doing internally to the robot log
		robot.debugMode(true);
	}

	@Override
	public void loop() {
		// By storing the gamepad state we can make sure that its values don't change during the loop
		Gamepad storedGamepad1 = gamepad1;
		Gamepad storedGamepad2 = gamepad2;

		robot.velocityDrive(storedGamepad1.right_stick_y, storedGamepad1.right_stick_x, storedGamepad1.left_stick_x, 2000, 28, 0.2);
		if (storedGamepad1.b) {
			robot.launcherVelocity(3000, 1, 28);
		} else {
			robot.launcherVelocity(0, 1, 28);
		}
	}
}
