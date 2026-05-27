package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.subsystem.BatteryVars.batteryVoltage;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
public class Turret implements Subsystem {
    public boolean shouldStop = true;
    private Turret() { }

    public final MotorEx motor1 = new MotorEx("turret1");
    private TelemetryManager telemetryM;

    // --- Control System Tuning ---
    public static PIDCoefficients pidCoefficients = new PIDCoefficients(0.05, 0, 0.001); // Tune these for ticks!
    public static BasicFeedforwardParameters feedforwardParameters = new BasicFeedforwardParameters(0, 0, 0);
    private final ControlSystem controlSystem = ControlSystem.builder()
            .posPid(pidCoefficients)
            .basicFF(feedforwardParameters)
            .build();

    // --- Cable Anti-Tangle Soft Limits (In Encoder Ticks) ---
    public static double MAX_ENCODER_TICKS = 1000;
    public static double MIN_ENCODER_TICKS = -1000;

    public static boolean manualOverride = false;

    // Changed from overrideSpeed to target encoder position override
    public static double overridePosition = 0;

    public boolean seesTag = false;

    public Command disableTurret = new InstantCommand(() -> manualOverride = true);
    public Command enableTurret = new InstantCommand(() -> manualOverride = false);

    public Command waitForTurret = new WaitUntil(() -> seesTag);

    public boolean isTurretInRange() {
        return Math.abs(Webcam.INSTANCE.lastOffset) < 5;
    }

    @Override
    public void initialize() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        motor1.zero();
    }

    @Override
    public void periodic() {
        telemetryM.addData("turret power", motor1.getPower());
        telemetryM.addData("turret position", motor1.getCurrentPosition());

        controlSystem.setGoal(new KineticState(0));
        if (manualOverride) {
            controlSystem.setGoal(new KineticState(overridePosition));
        }

        double power = controlSystem.calculate(motor1.getState());
        motor1.setPower(power);
    }

    public static final Turret INSTANCE = new Turret();
}