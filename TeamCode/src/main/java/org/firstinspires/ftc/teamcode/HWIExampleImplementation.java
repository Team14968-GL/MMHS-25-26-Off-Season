package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.HWI.*;
import com.qualcomm.hardware.lynx.commands.standard.LynxSetModuleLEDColorCommand;

@TeleOp
public class HWIExampleImplementation extends OpMode {
    HWI robot = new HWI(this);

    @Override
    public void init() {
        robot.init();
    }

    @Override
    public void loop() {
        robot.velocityDrive(gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, 2000, 28, 0.2);
    }
}
