package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;
import com.pedropathing.ivy.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

@SuppressWarnings("unused")
@TeleOp
public class IvyTest extends OpMode {

	private DcMotor leftLauncher, rightLauncher;
	private TouchSensor intakeBump1;

	private Command raiseArm;

	@Override
	public void init() {
		Scheduler.reset();

		leftLauncher = hardwareMap.get(DcMotor.class, "leftLauncher");
		rightLauncher = hardwareMap.get(DcMotor.class, "rightLauncher");
		intakeBump1 = hardwareMap.get(TouchSensor.class, "intakeBump1");

		raiseArm = Command.build()
				.setExecute(() -> launcher(0.5))
				.setDone(() -> !intakeBump1.isPressed())
				.setEnd(endCondition -> launcher(0))
				.requiring(leftLauncher, rightLauncher);



		Scheduler.schedule(raiseArm);
	}

	@Override
	public void loop() {
		Scheduler.execute();
		isScheduled(raiseArm);

	}
	private void launcher(double Power){
		leftLauncher.setPower(Power);
		rightLauncher.setPower(Power);
	}
	private void isScheduled(Command command) {
		if (command.isScheduled()) {
			telemetry.addData(command.toString(), command.isScheduled());
		}
	}
}
