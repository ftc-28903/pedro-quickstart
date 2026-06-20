package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.utils.AutoStorage.follower;
import static org.firstinspires.ftc.teamcode.subsystem.BatteryVars.batteryVoltage;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.MathFunctions;

import org.firstinspires.ftc.teamcode.utils.AllianceColor;
import org.firstinspires.ftc.teamcode.utils.AutoStorage;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
public class Turret implements Subsystem {
    private Turret() { }

    public final MotorEx motor1 = new MotorEx("turret1").reversed();

    private TelemetryManager telemetryM;

    // --- Control System Tuning ---
    public static PIDCoefficients pidCoefficients = new PIDCoefficients(0.003, 0, 0.00014);
    private final ControlSystem controlSystem = ControlSystem.builder()
            .posPid(pidCoefficients)
            .build();

    // --- Cable Anti-Tangle Soft Limits (In Encoder Ticks) ---
    public static double MAX_ENCODER_TICKS = 370;
    public static double MIN_ENCODER_TICKS = -370;

    public enum Mode {
        AUTO,
        MANUAL
    }
    public static Mode mode = Mode.AUTO;

    // --- Offset Feature ---
    // In AUTO, this modifies the absolute target angle.
    // In MANUAL, this explicitly dictates the physical target position.
    public static double offsetTicks = 0;

    public Command disableTurret = Commands.instant(() -> mode = Mode.MANUAL);
    public Command enableTurret = Commands.instant(() -> mode = Mode.AUTO);

    // TODO: deprecate
    public Command waitForTurret = Commands.instant(() -> {});

    public boolean isTurretInRange() {
        return true;
    }

    @Override
    public void initialize() {
        // TODO: fix zeroing condition
        if (!AutoStorage.prevOpmodeWasAuto && AutoStorage.isFirstRun) {
            motor1.zero();
        }
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    public static double TICKS_PER_RADIAN = 114.2;
    public static double targetX = 0.0;
    public static double targetY = 142.0;
    public static double kS = 0.08;

    // Convert 40mm to inches
    public static final double TURRET_OFFSET_INCHES = -40.0 / 25.4;

    /**
     * Calculates the relative yaw angle the turret must turn to look at a target,
     * factoring in the robot's heading and physical turret offset.
     */
    public double face(Pose targetPose, Pose robotPose) {
        // --- 40mm Offset Compensation ---
        double turretX = robotPose.getX() + (TURRET_OFFSET_INCHES * Math.cos(robotPose.getHeading()));
        double turretY = robotPose.getY() + (TURRET_OFFSET_INCHES * Math.sin(robotPose.getHeading()));

        // 1. Calculate the absolute world angle from the TURRET to the target
        double angleToTargetFromTurret = Math.atan2(targetPose.getY() - turretY, targetPose.getX() - turretX);

        // 2. Calculate angle relative to the robot's heading.
        double robotAngleDiff = angleToTargetFromTurret - robotPose.getHeading() + Math.PI;

        // 3. Normalize to shortest path (-PI to PI)
        return normalizeAngle(robotAngleDiff);
    }

    /**
     * Normalizes an angle into the range [-PI, PI]
     */
    public static double normalizeAngle(double angleRadians) {
        double angle = angleRadians % (Math.PI * 2D);
        if (angle <= -Math.PI) angle += Math.PI * 2D;
        if (angle > Math.PI) angle -= Math.PI * 2D;
        return angle;
    }

    /**
     * Increments the offsetTicks based on manual traveler/joystick inputs.
     * Use this method in your TeleOp to move the turret manually.
     *
     * @param amount The relative tick adjustment (e.g., gamepad1.right_stick_x * scale)
     */
    public void adjustOffset(double amount) {
        offsetTicks += amount;
        // Keep the offset accumulation bounded nicely within physical extremes
        offsetTicks = MathFunctions.clamp(offsetTicks, MIN_ENCODER_TICKS, MAX_ENCODER_TICKS);
    }

    @Override
    public void periodic() {
        if (AutoStorage.follower == null) return;
        if (!AutoStorage.opModeStarted) return;

        Pose robotPose = follower.getPose();
        Pose targetPose = new Pose(targetX, targetY, 0);
        if (AutoStorage.allianceColor == AllianceColor.RED) {
            targetPose = targetPose.mirror();
        }

        telemetryM.addData("robot x", robotPose.getX());
        telemetryM.addData("robot y", robotPose.getY());
        telemetryM.addData("heading", robotPose.getHeading());

        // Calculate target relative angle via face() method
        double steer = face(targetPose, robotPose);

        // Determine target position in encoder ticks
        double calculatedTargetTicks;
        if (mode == Mode.MANUAL) {
            // Read directly from offsetTicks instead of overridePosition
            calculatedTargetTicks = offsetTicks;
        } else {
            // In AUTO, offsetTicks acts as a travel-compensated modifier to the vector track
            calculatedTargetTicks = (steer * TICKS_PER_RADIAN) + offsetTicks;
        }

        // Apply Cable Anti-Tangle Soft Limits
        calculatedTargetTicks = MathFunctions.clamp(calculatedTargetTicks, MIN_ENCODER_TICKS, MAX_ENCODER_TICKS);
        calculatedTargetTicks = Math.round(calculatedTargetTicks);

        // Feed calculated ticks into the PID controller
        controlSystem.setGoal(new KineticState(calculatedTargetTicks));

        // Motor power calculation with kS and battery voltage compensation
        double rawPower = controlSystem.calculate(motor1.getState());
        rawPower += Math.signum(rawPower) * kS;
        double compensatedPower = rawPower * (13.0 / batteryVoltage);

        motor1.setPower(compensatedPower);

        // --- Telemetry ---
        telemetryM.addData("turret power", motor1.getPower());
        telemetryM.addData("turret position", motor1.getCurrentPosition());
        telemetryM.addData("turret goalPos", controlSystem.getGoal().getPosition());
        telemetryM.addData("turret offset ticks", offsetTicks);
        telemetryM.addData("turret steer error (deg)", Math.toDegrees(steer));
    }

    public static final Turret INSTANCE = new Turret();
}