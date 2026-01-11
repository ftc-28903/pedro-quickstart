package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name = "MecanumTest")
public class MecanumTest extends NextFTCOpMode {
    public MecanumTest() {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new SubsystemComponent(Shooter.INSTANCE)
        );
    }

    private final MotorEx frontLeftMotor = new MotorEx("front_left");
    private final MotorEx frontRightMotor = new MotorEx("front_right").reversed();
    private final MotorEx backLeftMotor = new MotorEx("back_left");
    private final MotorEx backRightMotor = new MotorEx("back_right").reversed();

    private TelemetryManager telemetryM;

    private double slowModeStep = 0.25;

    private boolean slowMode = false;
    private double slowModeMultiplier = 0.25;

    private Follower follower;
    public static Pose startingPose;

    @Override
    public void onInit() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void onStartButtonPressed() {
        follower.startTeleOpDrive();

        Gamepads.gamepad1().triangle().toggleOnBecomesTrue()
                .whenBecomesTrue(() -> slowMode = true)
                .whenBecomesFalse(() -> slowMode = false);

        Gamepads.gamepad1().dpadUp()
                .whenBecomesTrue(() ->
                        slowModeMultiplier = Math.min(1, slowModeMultiplier+slowModeStep));

        Gamepads.gamepad1().dpadDown()
                .whenBecomesTrue(() ->
                        slowModeMultiplier = Math.max(slowModeStep, slowModeMultiplier-slowModeStep));

        Gamepads.gamepad1().rightTrigger().greaterThan(0.2)
                .whenBecomesTrue(() -> Shooter.INSTANCE.spinUp.schedule())
                .whenBecomesFalse(() -> Shooter.INSTANCE.spinDown.schedule());
    }

    @Override
    public void onUpdate() {
        follower.update();
        telemetryM.update(telemetry);

        if (!slowMode) follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true // Robot Centric
        );
        else follower.setTeleOpDrive(
                -gamepad1.left_stick_y * slowModeMultiplier,
                -gamepad1.left_stick_x * slowModeMultiplier,
                -gamepad1.right_stick_x * slowModeMultiplier,
                true // Robot Centric
        );

        telemetryM.addData("slowMode toggle", slowMode);
        telemetryM.addData("slowMode multiplier", slowModeMultiplier);

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
    }
}