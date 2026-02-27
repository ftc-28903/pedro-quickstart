package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
@TeleOp(name = "TestOpmode")
public class TestOpmode extends NextFTCOpMode {
    public TestOpmode() {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    //private final IMUEx imu = new IMUEx("imu", Direction.UP, Direction.FORWARD);

    public final MotorEx motor1 = new MotorEx("shooter1").reversed();
    public final MotorEx motor2 = new MotorEx("shooter2");
    public final MotorEx intake1 = new MotorEx("intake1").reversed();
    public final MotorEx intake2 = new MotorEx("intake2").reversed();
    private final ElapsedTime loopTimeTimer = new ElapsedTime();

    private TelemetryManager telemetryM;

    private final double slowModeStep = 0.25;

    private boolean slowMode = false;
    private double slowModeMultiplier = 0.25;
    private boolean headingLock = false;
    private double headingLockPower = 0.0;

    public static boolean manualControl = false;
    public static double manualControlVelo = 0.0;
    public VoltageSensor voltageSensor;

    public static PIDCoefficients headingPIDCoefficients = new PIDCoefficients(0, 0, 0.0);
    private final ControlSystem controlSystem = ControlSystem.builder()
            .posPid(headingPIDCoefficients)
            .build();

    @Override
    public void onInit() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        voltageSensor = ActiveOpMode.hardwareMap().get(VoltageSensor.class, "Control Hub");
    }

    @Override
    public void onStartButtonPressed() {
        Gamepads.gamepad1().a().toggleOnBecomesTrue()
                .whenBecomesTrue(() -> {
                    motor1.setPower(0.3);
                    motor2.setPower(0.3);
                })
                .whenBecomesFalse(() -> {
                    motor1.setPower(0);
                    motor2.setPower(0);
                });

        Gamepads.gamepad1().rightTrigger().greaterThan(0.2)
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() -> intake1.setPower(1))
                .whenBecomesFalse(() -> intake1.setPower(0));

        Gamepads.gamepad1().leftTrigger().greaterThan(0.2)
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() -> intake2.setPower(1))
                .whenBecomesFalse(() -> intake2.setPower(0));
    }

    public double calculatePower(double velocity) {
        double m = 3.811e-04;
        double b = 1.420e-01;
        return m * velocity + b;
    }

    @Override
    public void onUpdate() {
        telemetryM.update(telemetry);
        telemetryM.addData("shootervelo", Math.abs(motor2.getVelocity()));
        double calcPower = calculatePower(manualControlVelo);
        telemetryM.addData("shooter2 motorSpeed", calcPower);

        if (manualControl) {
            double speed = (13 / voltageSensor.getVoltage()) * calcPower;
            telemetryM.addData("compensated power", speed);
            motor1.setPower(speed);
            motor2.setPower(speed);
        } else {
            motor1.setPower(0);
            motor2.setPower(0);
        }

        //hood1.setPosition(1);
    }
}