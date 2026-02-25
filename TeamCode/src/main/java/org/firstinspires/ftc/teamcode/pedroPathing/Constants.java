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
            .forwardZeroPowerAcceleration(-44.792129192214446) //deceleration on the forward axis
            .lateralZeroPowerAcceleration(-93.72450685019226) //deceleration on the lateral axis

            .translationalPIDFCoefficients(
                    new PIDFCoefficients(
                            0.1,
                            0,
                            0.00,
                            0.025
                    )
            )
            .headingPIDFCoefficients(
                    new PIDFCoefficients(
                            1.5,
                            0,
                            0.00,
                            0.01
                    )
            )
            .drivePIDFCoefficients(
                    new FilteredPIDFCoefficients(
                            0.1,
                            0.0,
                            0.01,
                            0.6,
                            0.0
                    )
            )
            
            ;

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightBack")
            .leftRearMotorName("leftBack")
            .leftFrontMotorName("leftFront")

            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)

            .xVelocity(79.59548157218873) //Velocity on the forward axis
            .yVelocity(52.49261234313485); //Velocity on the lateral axis

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(0.625000528) //inches from center of rotation
            .strafePodX(5.1349999548) //^^^
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD) //set accordingly
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
