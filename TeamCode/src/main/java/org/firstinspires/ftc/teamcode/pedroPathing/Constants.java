package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.8862) //mass in Kilos

            .forwardZeroPowerAcceleration(-44.792129192214446) //Deceleration on the forward axis
            .lateralZeroPowerAcceleration(-93.72450685019226) //Deceleration on the lateral axis

            .translationalPIDFCoefficients( //Adjusts how the robot moves side to side to correct it y-error
                    new PIDFCoefficients(
                            0.1,
                            0,
                            0.00,
                            0.025
                    )
            )
            .headingPIDFCoefficients( //Adjusts how the robot turns to correct it's heading error
                    new PIDFCoefficients(
                            1.5,
                            0,
                            0.00,
                            0.01
                    )
            )
            // TODO: 3/3/2026 everything below this point in follower constants in untuned and at default values
            .drivePIDFCoefficients( //Adjusts how the robot moves forwards and backwards to correct it's x-error? more testing needed to confirm
                    new FilteredPIDFCoefficients(
                            0.1,
                            0.0,
                            0.005,
                            0.6,
                            0.0
                    )
            )
            .centripetalScaling(0.005) //Adjusts how the robot follows curves
            ;

    public static MecanumConstants driveConstants = new MecanumConstants()
            //Sets robot's max power
            .maxPower(1)
            //Sets up motors in Pedro Pathing hardware map
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightBack")
            .leftRearMotorName("leftBack")
            .leftFrontMotorName("leftFront")
            //Sets up motor directions
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            //Sets the robot's max velocity on the x and y axis
            .xVelocity(79.59548157218873) //Velocity on the forward axis
            .yVelocity(52.49261234313485); //Velocity on the lateral axis

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(0.625000528 + 3) //Inches from center of rotation on it's respective axis
            .strafePodX(5.1349999548) //^^^
            .distanceUnit(DistanceUnit.INCH) //Unit for measuring distance
            .hardwareMapName("pinpoint") //Hardware map setup for Pedro Pathing localizer
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD) //Sets the encoder resolution to a preset for GoBilda Pinpoint
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD) //Direction that the encoder records data in
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED); //^^^
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
