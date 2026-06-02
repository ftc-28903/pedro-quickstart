package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.subsystem.BatteryVars.batteryVoltage;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;
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
    public static PIDCoefficients pidCoefficients = new PIDCoefficients(0.005, 0, 0.0002);
    public static BasicFeedforwardParameters feedforwardParameters = new BasicFeedforwardParameters(0, 0, 0);
    private final ControlSystem controlSystem = ControlSystem.builder()
            .posPid(pidCoefficients)
            .build();

    // --- Cable Anti-Tangle Soft Limits (In Encoder Ticks) ---
    public static double MAX_ENCODER_TICKS = 370;
    public static double MIN_ENCODER_TICKS = -370;

    public static boolean manualOverride = false;
    public static double overridePosition = 0;

    // --- Offset Feature ---
    // Change this value via telemetry/dashboard or commands to adjust the 0-degree point
    public static double offsetTicks = 0;

    public boolean seesTag = false;

    public Command disableTurret = Commands.instant(() -> manualOverride = true);
    public Command enableTurret = Commands.instant(() -> manualOverride = false);

    // Commands to add/subtract from the offset dynamically (e.g., binding to gamepads)
    /*public Command incrementOffset(double ticks) {
        return new InstantCommand(() -> offsetTicks += ticks);
    }
    public Command decrementOffset(double ticks) {
        return new InstantCommand(() -> offsetTicks -= ticks);
    }*/

    // TODO: deprecate
    public Command waitForTurret = Commands.instant(() -> {});

    public static double kF = 0.08;

    public boolean isTurretInRange() {
        return Math.abs(Webcam.INSTANCE.lastOffset) < 5;
    }

    @Override
    public void initialize() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        motor1.zero();
    }

    public static double TICKS_PER_RADIAN = 100;

    @Override
    public void periodic() {
        double x = follower().getPose().getX();
        double y = follower().getPose().getY();
        double heading = follower().getPose().getHeading();
        telemetryM.addData("x", x);
        telemetryM.addData("y", y);
        telemetryM.addData("heading", heading);

        // 1. Target heading is locked at 0 degrees (0 radians)
        double targetWorldHeading = 0.0;

        // 2. Calculate the angle relative to the robot's current face.
        // Because the physical zero position faces away from the front of the robot,
        // we subtract Math.PI (180 degrees) to rotate the reference frame forward.
        double steer = targetWorldHeading - heading - Math.PI;

        // 3. Normalize steer to the shortest path (-PI to PI) using atan2 for clean wrapping
        steer = Math.atan2(Math.sin(steer), Math.cos(steer));

        // 4. Determine target position in encoder ticks
        double calculatedTargetTicks;

        if (manualOverride) {
            calculatedTargetTicks = overridePosition;
        } else {
            // Base calculation to point at 0 degrees + user offset
            calculatedTargetTicks = (steer * TICKS_PER_RADIAN) + offsetTicks;
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
        telemetryM.addData("turret offset ticks", offsetTicks);
        telemetryM.addData("turret steer error (deg)", Math.toDegrees(steer));
    }

    public static final Turret INSTANCE = new Turret();
}