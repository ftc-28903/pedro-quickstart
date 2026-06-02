package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.subsystem.BatteryVars.batteryVoltage;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

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

    public final MotorEx motor1 = new MotorEx("turret1").reversed();
    private TelemetryManager telemetryM;

    // --- Control System Tuning ---
    public static PIDCoefficients pidCoefficients = new PIDCoefficients(0.005, 0, 0.0002); // Tune these for ticks!
    public static BasicFeedforwardParameters feedforwardParameters = new BasicFeedforwardParameters(0, 0, 0);
    private final ControlSystem controlSystem = ControlSystem.builder()
            .posPid(pidCoefficients)
            .build();

    // --- Cable Anti-Tangle Soft Limits (In Encoder Ticks) ---
    public static double MAX_ENCODER_TICKS = 370;
    public static double MIN_ENCODER_TICKS = -370;

    public static boolean manualOverride = false;

    // Changed from overrideSpeed to target encoder position override
    public static double overridePosition = 0;

    public boolean seesTag = false;

    public Command disableTurret = new InstantCommand(() -> manualOverride = true);
    public Command enableTurret = new InstantCommand(() -> manualOverride = false);

    public Command waitForTurret = new WaitUntil(() -> seesTag);

    public static double kF = 0.08;

    public boolean isTurretInRange() {
        return Math.abs(Webcam.INSTANCE.lastOffset) < 5;
    }

    @Override
    public void initialize() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        motor1.zero();
    }

    double targetX = 0;
    double targetY = 144;

    public static double TICKS_PER_RADIAN=100;

    @Override
    public void periodic() {
        double x = follower().getPose().getX();
        double y = follower().getPose().getY();
        double heading = follower().getPose().getHeading();
        telemetryM.addData("x", x);
        telemetryM.addData("y", y);
        telemetryM.addData("heading", heading);

        // 1. Calculate the target angle in the global world map
        double target = Math.atan2(targetY - y, targetX - x);

        // 2. Calculate the angle relative to the robot's current face (0 = Straight Forward)
        double steer = target - heading;

        // 3. Normalize steer to the shortest path (-PI to PI)
        while (steer > Math.PI)  steer -= 2 * Math.PI;
        while (steer < -Math.PI) steer += 2 * Math.PI;

        // 4. Determine target position in encoder ticks
        double calculatedTargetTicks;

        if (manualOverride) {
            calculatedTargetTicks = overridePosition;
        } else {
            // Because 0 ticks is straight forward, the 'steer' angle *is* our absolute target angle!
            // We multiply it directly by TICKS_PER_RADIAN instead of adding to current position.
            calculatedTargetTicks = steer * TICKS_PER_RADIAN;
        }

        // 5. Apply your Cable Anti-Tangle Soft Limits
        calculatedTargetTicks = Math.max(MIN_ENCODER_TICKS, Math.min(MAX_ENCODER_TICKS, calculatedTargetTicks));

        // 6. Feed the calculated ticks into your PID controller
        controlSystem.setGoal(new KineticState(calculatedTargetTicks));

        // 7. Motor power calculation
        double rawPower = controlSystem.calculate(motor1.getState());
        double compensatedPower = rawPower * (13.0 / batteryVoltage);

        double ffPower = 0;
        if (Math.abs(rawPower) > 0.01) {
            ffPower = Math.signum(compensatedPower) * kF + compensatedPower;
        }

        motor1.setPower(ffPower);

        // --- Telemetry ---
        telemetryM.addData("turret power", motor1.getPower());
        telemetryM.addData("turret position", motor1.getCurrentPosition());
        telemetryM.addData("turret goalPos", controlSystem.getGoal().getPosition());
        telemetryM.addData("turret steer error (deg)", Math.toDegrees(steer));
    }

    public static final Turret INSTANCE = new Turret();
}