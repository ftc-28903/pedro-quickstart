package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.auto.TrajectoryFactory;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.Transfer;
import org.firstinspires.ftc.teamcode.subsystem.Turret;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Configurable
@TeleOp(name="PedroTest")
public class PedroTest extends NextFTCOpMode {
    public PedroTest() {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new SubsystemComponent(Shooter.INSTANCE, Intake.INSTANCE, Transfer.INSTANCE, Turret.INSTANCE)
        );
    }

    private TelemetryManager telemetryM;
    public static Follower follower;
    public static Pose startingPose;

    public boolean slowMode = false;
    public double slowModeMultiplier = 0.6;

    @Override
    public void onInit() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? TrajectoryFactory.INSTANCE.startPose : startingPose);
        follower.update();
        Turret.INSTANCE.follower = follower;

        Intake.INSTANCE.spinDown.schedule();
        Shooter.INSTANCE.spinDown.schedule();
        Transfer.INSTANCE.overrideOff.schedule();
        Transfer.INSTANCE.opModeOverrideOff.schedule();

        Transfer.INSTANCE.offOverrideOn.schedule();

        Turret.INSTANCE.disableTurret.schedule();
    }

    @Override
    public void onStartButtonPressed() {
        Turret.INSTANCE.enableTurret.schedule();
        Transfer.INSTANCE.offOverrideOff.schedule();

        Gamepads.gamepad1().circle().whenBecomesTrue(() -> slowMode = !slowMode);
        Gamepads.gamepad1().dpadUp().whenBecomesTrue(() -> slowModeMultiplier = Math.min(1.0, slowModeMultiplier+0.1));
        Gamepads.gamepad1().dpadDown().whenBecomesTrue(() -> slowModeMultiplier = Math.max(0.0, slowModeMultiplier-0.1));

        Gamepads.gamepad1().share().toggleOnBecomesTrue()
                .whenBecomesTrue(() -> Transfer.INSTANCE.offOverrideOn.schedule())
                .whenBecomesFalse(() -> Transfer.INSTANCE.offOverrideOff.schedule());

        Gamepads.gamepad1().cross().toggleOnBecomesTrue()
                .whenBecomesTrue(() -> Shooter.INSTANCE.spinUp.schedule())
                .whenBecomesFalse(() -> Shooter.INSTANCE.spinDown.schedule());

        Gamepads.gamepad1().leftTrigger().greaterThan(0.6).toggleOnBecomesTrue()
                .whenBecomesTrue(() -> Intake.INSTANCE.spinUp.schedule())
                .whenBecomesFalse(() -> Intake.INSTANCE.spinDown.schedule());

        Gamepads.gamepad1().rightTrigger().greaterThan(0.6)
                .whenBecomesTrue(() -> Transfer.INSTANCE.overrideOn.schedule())
                .whenBecomesFalse(() -> Transfer.INSTANCE.overrideOff.schedule());

        follower.startTeleOpDrive(true);
    }

    @Override
    public void onUpdate() {
        follower.update();
        telemetryM.update();
        Scheduler.execute();

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
    }
}
