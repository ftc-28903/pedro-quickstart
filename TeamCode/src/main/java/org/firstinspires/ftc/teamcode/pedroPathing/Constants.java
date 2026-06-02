package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.065, 0.08659, 0.0016472))
            .headingPIDFCoefficients(new PIDFCoefficients(1.9, 0, 0.075, 0.01))
            .centripetalScaling(0)
            .mass(10);

    public static PathConstraints pathConstraints = new PathConstraints(0.9,100,1,1);

    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardEncoder_HardwareMapName("back_left")
            .strafeEncoder_HardwareMapName("back_right")
            .strafeEncoderDirection(Encoder.REVERSE)
            .forwardTicksToInches(11.754E-4)
            .strafeTicksToInches(11.754E-4)
            .forwardPodY(-7.23)
            .strafePodX(0)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.UP,
                            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                    )
            );

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .xVelocity(82.5)
            .yVelocity(57.4)
            .rightFrontMotorName("front_right")
            .rightRearMotorName("back_right")
            .leftFrontMotorName("front_left")
            .leftRearMotorName("back_left")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pathConstraints(pathConstraints)
                .twoWheelLocalizer(localizerConstants)
                .build();
    }
}
