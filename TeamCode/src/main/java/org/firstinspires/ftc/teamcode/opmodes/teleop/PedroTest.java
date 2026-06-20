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
import org.firstinspires.ftc.teamcode.subsystem.Webcam;
import org.firstinspires.ftc.teamcode.utils.AllianceColor;
import org.firstinspires.ftc.teamcode.utils.AutoStorage;
import org.firstinspires.ftc.teamcode.utils.CGHelpers;
import org.firstinspires.ftc.teamcode.utils.ShooterRegression;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.ActiveOpMode;
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
                new SubsystemComponent(Shooter.INSTANCE, Intake.INSTANCE, Transfer.INSTANCE, Turret.INSTANCE, Webcam.INSTANCE)
        );
    }

    private TelemetryManager telemetryM;
    public static Follower follower;
    public static Pose startingPose;

    public boolean slowMode = false;
    public double slowModeMultiplier = 0.6;

    // --- Targeted Slew Rate Limiter Setup ---
    private double limitedLeftStickY = 0.0;
    private long lastUpdateTime = 0;

    // Limits the violent forward spike when crossing over from reverse
    public static double MAX_RATE_OF_CHANGE = 3.0;

    @Override
    public void onInit() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        if (AutoStorage.allianceColor == AllianceColor.RED) {
            follower.setStartingPose(AutoStorage.autoEndPose == null ? TrajectoryFactory.INSTANCE.startPose.mirror() : AutoStorage.autoEndPose);
        } else {
            follower.setStartingPose(AutoStorage.autoEndPose == null ? TrajectoryFactory.INSTANCE.startPose : AutoStorage.autoEndPose);
        }
        follower.update();
        AutoStorage.follower = follower;
        CGHelpers.getInitGroup().schedule();

        Webcam.INSTANCE.init();

        AutoStorage.opModeStarted = false;
    }

    @Override
    public void onStartButtonPressed() {
        AutoStorage.isFirstRun = false;
        AutoStorage.opModeStarted = true;

        CGHelpers.getStartGroup().schedule();
        Turret.INSTANCE.enableTurret.schedule();

        Gamepads.gamepad1().circle().whenBecomesTrue(() -> slowMode = !slowMode);
        Gamepads.gamepad1().dpadUp().whenBecomesTrue(() -> slowModeMultiplier = Math.min(1.0, slowModeMultiplier+0.1));
        Gamepads.gamepad1().dpadDown().whenBecomesTrue(() -> slowModeMultiplier = Math.max(0.0, slowModeMultiplier-0.1));

        Gamepads.gamepad2().cross().toggleOnBecomesTrue()
                .whenBecomesTrue(() -> Shooter.INSTANCE.spinUp.schedule())
                .whenBecomesFalse(() -> Shooter.INSTANCE.spinDown.schedule());

        Gamepads.gamepad1().leftTrigger().greaterThan(0.6).toggleOnBecomesTrue()
                .whenBecomesTrue(() -> Intake.INSTANCE.spinUp.schedule())
                .whenBecomesFalse(() -> Intake.INSTANCE.spinDown.schedule());

        Gamepads.gamepad1().rightTrigger().greaterThan(0.6)
                .whenBecomesTrue(() -> Transfer.INSTANCE.overrideOn.schedule())
                .whenBecomesFalse(() -> Transfer.INSTANCE.overrideOff.schedule());

        Gamepads.gamepad2().ps()
                .whenBecomesTrue(() -> {
                    follower.setPose(AutoStorage.allianceColor == AllianceColor.BLUE ?
                            new Pose(8.5, 8.5, Math.toRadians(180)) :
                            new Pose(8.5, 8.5, Math.toRadians(180)).mirror());
                    follower.update(); // Prevent immediate localization twitching
                });

        Gamepads.gamepad2().circle()
                .whenBecomesTrue(() -> {
                    follower.setPose(AutoStorage.allianceColor == AllianceColor.BLUE ?
                            new Pose(15.5, 112.4, Math.toRadians(180)) :
                            new Pose(15.5, 112.4, Math.toRadians(180)).mirror());
                    follower.update();
                });

        Gamepads.gamepad2().triangle()
                .whenBecomesTrue(() -> {
                    follower.setPose(AutoStorage.allianceColor == AllianceColor.BLUE ?
                            new Pose(36.8, 133.0, Math.toRadians(90)) :
                            new Pose(36.8, 133.0, Math.toRadians(90)).mirror());
                    follower.update();
                });

        Gamepads.gamepad2().share().toggleOnBecomesTrue()
                .whenBecomesTrue(() -> Transfer.INSTANCE.offOverrideOn.schedule())
                .whenBecomesFalse(() -> Transfer.INSTANCE.offOverrideOff.schedule());

        Gamepads.gamepad2().rightBumper()
                .whenBecomesTrue(() -> ShooterRegression.GLOBAL_VELOCITY_OFFSET+=10);

        Gamepads.gamepad2().leftBumper()
                .whenBecomesTrue(() -> ShooterRegression.GLOBAL_VELOCITY_OFFSET-=10);

        Gamepads.gamepad2().rightBumper().and(Gamepads.gamepad2().leftBumper())
                .whenBecomesTrue(() -> ShooterRegression.GLOBAL_VELOCITY_OFFSET = 0);

        Gamepads.gamepad2().dpadLeft()
                .whenBecomesTrue(() -> {
                    double increment = Shooter.teleOpOverride ? 20 : 5;
                    // On Blue, Left is +. On Red, the field is mirrored, so Left becomes -
                    double allianceModifier = (AutoStorage.allianceColor == AllianceColor.BLUE ? 1 : -1);
                    Turret.INSTANCE.adjustOffset(increment * allianceModifier);
                });

        Gamepads.gamepad2().dpadRight()
                .whenBecomesTrue(() -> {
                    double increment = Shooter.teleOpOverride ? 20 : 5;
                    // On Blue, Right is -. On Red, the field is mirrored, so Right becomes +
                    double allianceModifier = (AutoStorage.allianceColor == AllianceColor.BLUE ? -1 : 1);
                    Turret.INSTANCE.adjustOffset(increment * allianceModifier);
                });

        Gamepads.gamepad2().dpadUp().toggleOnBecomesTrue()
                .whenBecomesTrue(() -> {
                    Turret.offsetTicks = 0;
                    Turret.INSTANCE.disableTurret.schedule();
                    Shooter.teleOpOverride = true;
                })
                .whenBecomesFalse(() -> {
                    Turret.offsetTicks = 0;
                    Turret.INSTANCE.enableTurret.schedule();
                    Shooter.teleOpOverride = false;
                });

        Gamepads.gamepad2().rightTrigger().greaterThan(0.8)
                .whenBecomesTrue(() -> Transfer.INSTANCE.forcePush.schedule())
                .whenBecomesFalse(() -> Transfer.INSTANCE.forcePushStop.schedule());

        Gamepads.gamepad2().leftTrigger().greaterThan(0.8)
                .whenBecomesTrue(() -> Transfer.INSTANCE.forcePushBack.schedule())
                .whenBecomesFalse(() -> Transfer.INSTANCE.forcePushBackStop.schedule());

        lastUpdateTime = System.nanoTime();

        follower.startTeleOpDrive(true);
    }

    @Override
    public void onUpdate() {
        follower.update();
        Scheduler.execute();

        // Calculate delta time
        long currentTime = System.nanoTime();
        double deltaTime = (currentTime - lastUpdateTime) / 1e9;
        lastUpdateTime = currentTime;

        // Apply translation inputs to Pedro Pathing follower
        if (!slowMode) {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // Robot Centric
            );
        } else {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    true // Robot Centric
            );
        }

        telemetryM.update(telemetry);
    }

    @Override
    public void onStop() {
        AutoStorage.prevOpmodeWasAuto = false;
    }
}