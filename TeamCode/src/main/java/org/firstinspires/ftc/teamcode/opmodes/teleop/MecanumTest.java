package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.Transfer;
import org.firstinspires.ftc.teamcode.subsystem.Webcam;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name = "MecanumTest")
public class MecanumTest extends NextFTCOpMode {
    public MecanumTest() {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new SubsystemComponent(Shooter.INSTANCE, Intake.INSTANCE, Transfer.INSTANCE, Webcam.INSTANCE)
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

    @Override
    public void onInit() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        Webcam.INSTANCE.init();
    }

    @Override
    public void onStartButtonPressed() {
        Command driverControlled = new MecanumDriverControlled(
            frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                Gamepads.gamepad1().leftStickY().negate().map(y -> slowMode ? y * slowModeMultiplier : y),
                Gamepads.gamepad1().leftStickX().map(x -> slowMode ? x * slowModeMultiplier : x),
                Gamepads.gamepad1().rightStickX().map(x -> slowMode ? x * slowModeMultiplier : x)
        );
        driverControlled.schedule();

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

        Gamepads.gamepad1().leftTrigger().greaterThan(0.2)
                .whenBecomesTrue(() -> Intake.INSTANCE.spinUp.schedule())
                .whenBecomesFalse(() -> Intake.INSTANCE.spinDown.schedule());

        Gamepads.gamepad1().leftBumper()
                .whenBecomesTrue(() -> Intake.INSTANCE.spinUpReverse.schedule())
                .whenBecomesFalse(() -> Intake.INSTANCE.spinDown.schedule());

        Gamepads.gamepad1().cross()
                .whenBecomesTrue(() -> Transfer.INSTANCE.overrideOn.schedule())
                .whenBecomesFalse(() -> Transfer.INSTANCE.overrideOff.schedule());

        Gamepads.gamepad1().share().toggleOnBecomesTrue()
                .whenBecomesTrue(() -> Transfer.INSTANCE.offOverrideOn.schedule())
                .whenBecomesFalse(() -> Transfer.INSTANCE.offOverrideOff.schedule());

        Gamepads.gamepad1().rightBumper()
                .whenBecomesTrue(() -> {
                    Shooter.shooterGoal = 1650;
                    Shooter.shooterAngle = 0.6;
                })
                .whenBecomesFalse(() -> {
                    Shooter.shooterGoal = 1350;
                    Shooter.shooterAngle = 0.4;
                });
    }

    @Override
    public void onUpdate() {
        telemetryM.update(telemetry);

        telemetryM.addData("slowMode toggle", slowMode);
        telemetryM.addData("slowMode multiplier", slowModeMultiplier);
    }
}