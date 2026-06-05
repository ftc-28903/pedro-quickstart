package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.subsystem.BatteryVars.batteryVoltage;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.MathFunctions;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
public class Turret implements Subsystem {
    public boolean shouldStop = true;
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

    public static boolean manualOverride = false;
    public static double overridePosition = 0;

    // --- Offset Feature ---
    public static double offsetTicks = 0;

    public boolean seesTag = false;

    public Command disableTurret = Commands.instant(() -> manualOverride = true);
    public Command enableTurret = Commands.instant(() -> manualOverride = false);

    // TODO: deprecate
    public Command waitForTurret = Commands.instant(() -> {});

    public boolean isTurretInRange() {
        return Math.abs(Webcam.INSTANCE.lastOffset) < 5;
    }

    @Override
    public void initialize() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        motor1.zero();
    }

    public static double TICKS_PER_RADIAN = 109.1;
    public static double targetX = 2.0;
    public static double targetY = 139.5;
    public Follower follower;

    public static double kS = 0.08;

    // Convert 40mm to inches
    public static final double TURRET_OFFSET_INCHES = -40.0 / 25.4;

    /**
     * Calculates the relative yaw angle the turret must turn to look at a target,
     * factoring in the robot's heading and physical turret offset.
     */
    public double face(Pose targetPose, Pose robotPose) {
        // --- 40mm Offset Compensation ---
        // Adjusts robot center to actual turret center of rotation
        double turretX = robotPose.getX() + (TURRET_OFFSET_INCHES * Math.cos(robotPose.getHeading()));
        double turretY = robotPose.getY() + (TURRET_OFFSET_INCHES * Math.sin(robotPose.getHeading()));

        // 1. Calculate the absolute world angle from the TURRET to the target
        double angleToTargetFromTurret = Math.atan2(targetPose.getY() - turretY, targetPose.getX() - turretX);

        // 2. Calculate angle relative to the robot's heading.
        // Adding Math.PI preserves your original physical mapping (e.g., zero faces away from the front).
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

    @Override
    public void periodic() {
        if (follower == null) return;

        Pose robotPose = follower.getPose();
        Pose targetPose = new Pose(targetX, targetY, 0);

        telemetryM.addData("robot x", robotPose.getX());
        telemetryM.addData("robot y", robotPose.getY());
        telemetryM.addData("heading", robotPose.getHeading());

        // Calculate target relative angle via face() method
        double steer = face(targetPose, robotPose);

        // Determine target position in encoder ticks
        double calculatedTargetTicks;
        if (manualOverride) {
            calculatedTargetTicks = overridePosition;
        } else {
            calculatedTargetTicks = (steer * TICKS_PER_RADIAN) + offsetTicks;
        }

        // Apply Cable Anti-Tangle Soft Limits
        calculatedTargetTicks = MathFunctions.clamp(calculatedTargetTicks, MIN_ENCODER_TICKS, MAX_ENCODER_TICKS);

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