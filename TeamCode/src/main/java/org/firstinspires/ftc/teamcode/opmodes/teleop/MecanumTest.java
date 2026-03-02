package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.Transfer;
import org.firstinspires.ftc.teamcode.subsystem.Turret;
import org.firstinspires.ftc.teamcode.subsystem.Webcam;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
@TeleOp(name = "MecanumTest")
public class MecanumTest extends NextFTCOpMode {
    public MecanumTest() {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new SubsystemComponent(Shooter.INSTANCE, Intake.INSTANCE, Transfer.INSTANCE, Webcam.INSTANCE, Turret.INSTANCE)
        );
    }

    private final IMUEx imu = new IMUEx("imu", Direction.UP, Direction.FORWARD);

    private final ElapsedTime loopTimeTimer = new ElapsedTime();

    private final MotorEx frontLeftMotor = new MotorEx("front_left").brakeMode();
    private final MotorEx frontRightMotor = new MotorEx("front_right").reversed().brakeMode();
    private final MotorEx backLeftMotor = new MotorEx("back_left").brakeMode();
    private final MotorEx backRightMotor = new MotorEx("back_right").reversed().brakeMode();

    private TelemetryManager telemetryM;

    private final double slowModeStep = 0.25;

    private boolean slowMode = false;
    private double slowModeMultiplier = 0.25;
    private boolean headingLock = false;
    private double headingLockPower = 0.0;

    public static PIDCoefficients headingPIDCoefficients = new PIDCoefficients(0.03, 0, 0.0);
    private final ControlSystem controlSystem = ControlSystem.builder()
            .posPid(headingPIDCoefficients)
            .build();

    @Override
    public void onInit() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        Webcam.INSTANCE.init();

        Intake.INSTANCE.spinDown.schedule();
        Shooter.INSTANCE.spinDown.schedule();
        Transfer.INSTANCE.overrideOff.schedule();
        Transfer.INSTANCE.opModeOverrideOff.schedule();

        Transfer.INSTANCE.offOverrideOn.schedule();
    }

    @Override
    public void onStartButtonPressed() {
        Transfer.INSTANCE.offOverrideOff.schedule();
        //Shooter.INSTANCE.spinUp.schedule();
        Command driverControlled = new MecanumDriverControlled(
            frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                Gamepads.gamepad1().leftStickY().negate().map(y -> slowMode ? y * slowModeMultiplier : y),
                Gamepads.gamepad1().leftStickX().map(x -> slowMode ? x * slowModeMultiplier : x),
                Gamepads.gamepad1().rightStickX().map(x -> (
                        headingLock ? headingLockPower : (slowMode ? x * slowModeMultiplier : x)
                        ))
                //new FieldCentric(imu)
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
                .whenBecomesTrue(() -> Transfer.INSTANCE.overrideOn.schedule())
                .whenBecomesFalse(() -> Transfer.INSTANCE.overrideOff.schedule());

        Gamepads.gamepad1().leftTrigger().greaterThan(0.2).toggleOnBecomesTrue()
                .whenBecomesTrue(() -> Intake.INSTANCE.spinUp.schedule())
                .whenBecomesFalse(() -> Intake.INSTANCE.spinDown.schedule());

        Gamepads.gamepad1().leftBumper()
                .whenBecomesTrue(() -> Intake.INSTANCE.spinUpReverse.schedule())
                .whenBecomesFalse(() -> Intake.INSTANCE.spinDown.schedule());

        Gamepads.gamepad1().cross().toggleOnBecomesTrue()
                .whenBecomesTrue(() -> Shooter.INSTANCE.spinUp.schedule())
                .whenBecomesFalse(() -> Shooter.INSTANCE.spinDown.schedule());

        Gamepads.gamepad1().triangle()
                        .whenBecomesTrue(imu::zero);

        Gamepads.gamepad1().share().toggleOnBecomesTrue()
                .whenBecomesTrue(() -> Transfer.INSTANCE.offOverrideOn.schedule())
                .whenBecomesFalse(() -> Transfer.INSTANCE.offOverrideOff.schedule());
    }

    @Override
    public void onUpdate() {
        telemetryM.update(telemetry);

        /*double offset = Webcam.INSTANCE.imuOffset;
        controlSystem.setGoal(new KineticState(0));
        headingLockPower = controlSystem.calculate(
                new KineticState(offset, Double.MAX_VALUE, Double.MAX_VALUE)
        );*/

        //telemetryM.addData("slowMode toggle", slowMode);
        telemetryM.addData("slowMode multiplier", slowModeMultiplier);
        telemetryM.addData("loop time", loopTimeTimer.milliseconds());
        telemetryM.addData("currentHeading", imu.get().inDeg);
        loopTimeTimer.reset();
    }
}