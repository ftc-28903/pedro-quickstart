package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.subsystem.BatteryVars.batteryVoltage;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.utils.AutoStorage;
import org.firstinspires.ftc.teamcode.utils.ShooterRegression;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

@Configurable
public class Shooter implements Subsystem {
    public enum State {
        RUNNING,
        STOPPED
    }
    public State state = State.STOPPED;
    private Shooter() { }

    public VoltageSensor voltageSensor;
    public final MotorEx motor1 = new MotorEx("shooter1").reversed();
    private final ServoEx hoodServo1 = new ServoEx("hood1");

    // --- Added ShooterRegression Instance ---
    private final ShooterRegression regression = new ShooterRegression();

    // --- Manual Velocity Tracking Variables ---
    private double lastTicks = 0;
    private long lastTimeNanos = 0;
    private double manualVelocityTicksPerSec = 0;
    // ------------------------------------------

    public static double shooterGoal = 1250;
    public static double shooterPowerOverride = 0.1;
    public static double shooterAngle = 0.58;
    public static PIDCoefficients pidCoefficients = new PIDCoefficients(0.0024, 0, 0.0);
    public static double velocityTolerance = 200;
    public static double voltageCalibration = 13.0;
    private TelemetryManager telemetryM;

    private final ControlSystem controlSystem = ControlSystem.builder()
            .velPid(pidCoefficients)
            .build();

    // Target constants mirrored from your Turret subsystem
    public static double targetX = 2.0;
    public static double targetY = 139.5;
    public static final double TURRET_OFFSET_INCHES = -40.0 / 25.4;

    public static double min_rpm_dynamic = 1070;

    public static boolean angleOverride = true;

    // --- Added Freeze State Flag ---
    public static boolean isFrozen = false;

    public Command spinUp = Commands.instant(() -> {
        state = State.RUNNING;
    });

    public Command spinDown = Commands.instant(() -> {
        state = State.STOPPED;
    });

    // --- Added Freeze / Unfreeze Commands ---
    public Command freeze = Commands.instant(() -> {
        isFrozen = true;
    });

    public Command unfreeze = Commands.instant(() -> {
        isFrozen = false;
    });

    public Command waitForSpeed = Command.build()
            .setDone(this::isSpeedGood);

    // ticksPerSecond = RPM x 28 / 60
    public double ticksToRPM(double ticksPerSecond, double countsPerRevolution) {
        return (ticksPerSecond / countsPerRevolution * 60);
    }
    public double rpmToTicks(double rpm, double countsPerRevolution) {
        return (rpm * countsPerRevolution / 60);
    }

    public boolean isSpeedGood() {
        if (state == State.STOPPED || mode == Mode.MANUAL) return true;
        double speed = Math.abs(manualVelocityTicksPerSec);

        return speed >= min_rpm_dynamic;
    }

    /**
     * Calculates y for a cubic equation using Horner's Method
     */
    public static double hoodPercentageToServo(double x) {
        final double a = 1.058e-06;
        final double b = -1.276e-04;
        final double c = 1.098e-02;
        final double d = 1.064e-01;

        double clampedX = Math.max(0, Math.min(100, x));
        return ((a * clampedX + b) * clampedX + c) * clampedX + d;
    }

    @Override
    public void initialize() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        voltageSensor = ActiveOpMode.hardwareMap().get(VoltageSensor.class, "Control Hub");

        hoodServo1.getServo().setDirection(Servo.Direction.FORWARD);
        PwmControl hoodServo1PWM = (PwmControl) hoodServo1.getServo();
        hoodServo1PWM.setPwmRange(new PwmControl.PwmRange(1100, 1800, 10000));

        lastTicks = motor1.getCurrentPosition();
        lastTimeNanos = System.nanoTime();
    }

    public enum Mode {
        AUTO,
        MANUAL
    }
    public static Mode mode = Mode.AUTO;
    public static double overrideVelo = 0;
    public static double overrideAngle = 0.5;

    public double calculatePower(double velocity) {
        double m = 3.632e-04;
        double b = 1.387e-01;
        return m * velocity + b;
    }

    @Override
    public void periodic() {
        // --- 1. Manual Ticks/Second Calculation ---
        long currentTimeNanos = System.nanoTime();
        double currentTicks = motor1.getCurrentPosition();

        long deltaTimeNanos = currentTimeNanos - lastTimeNanos;

        if (deltaTimeNanos > 0) {
            double deltaTicks = currentTicks - lastTicks;
            double deltaTimeSeconds = deltaTimeNanos / 1_000_000_000.0;
            manualVelocityTicksPerSec = deltaTicks / deltaTimeSeconds;
        }

        manualVelocityTicksPerSec = motor1.getVelocity();

        lastTicks = currentTicks;
        lastTimeNanos = currentTimeNanos;
        // ------------------------------------------

        batteryVoltage = voltageSensor.getVoltage();
        telemetryM.addData("batteryVoltage", batteryVoltage);
        telemetryM.addData("isFrozen", isFrozen);

        if (AutoStorage.follower != null) {
            // Fetch live odometry data from the follower
            Pose robotPose = AutoStorage.follower.getPose();

            // --- 40mm Offset Compensation ---
            // Determines where the center of the turret/shooter sits in world coordinates
            double turretX = robotPose.getX() + (TURRET_OFFSET_INCHES * Math.cos(robotPose.getHeading()));
            double turretY = robotPose.getY() + (TURRET_OFFSET_INCHES * Math.sin(robotPose.getHeading()));

            // --- 2D Euclidean Distance Calculation ---
            double deltaX = targetX - turretX;
            double deltaY = targetY - turretY;
            double distanceInches = Math.hypot(deltaX, deltaY);

            min_rpm_dynamic = regression.getMinRpmForDistance(distanceInches);

            telemetryM.addData("robotX", robotPose.getX());
            telemetryM.addData("robotY", robotPose.getY());
            telemetryM.addData("turretCalculatedX", turretX);
            telemetryM.addData("turretCalculatedY", turretY);
            telemetryM.addData("shooterDistanceInches", distanceInches);

            // Only update targets if values are not currently frozen
            if (!isFrozen) {
                // 1. Calculate Target RPM using ShooterRegression and convert to native controller units
                double calculatedRPM = regression.getTargetRpm(distanceInches);

                // 2. Adjust hood scaling dynamically using the current actual flywheel RPM via ShooterRegression
                double calculatedHoodPct = regression.calculateDynamicHood(distanceInches, manualVelocityTicksPerSec);
                double calculatedServoPos = hoodPercentageToServo(calculatedHoodPct);

                shooterAngle = calculatedServoPos;
                shooterGoal = calculatedRPM;

                telemetryM.addData("shooterTicksEstimate", calculatedRPM);
                telemetryM.addData("shooterHoodPctEstimate", calculatedHoodPct);
                telemetryM.addData("shooterServoEstimate", calculatedServoPos);
            }

            if (mode == Mode.MANUAL) {
                shooterGoal = overrideVelo;
                shooterAngle = angleOverride ? hoodPercentageToServo(overrideAngle) : shooterAngle;
            }
            controlSystem.setGoal(new KineticState(Double.MAX_VALUE, shooterGoal, Double.MAX_VALUE));
        }

        hoodServo1.setPosition(shooterAngle);

        double rawPower = 1;
        if ((Math.abs(manualVelocityTicksPerSec)-Math.abs(shooterGoal))>40) {
            rawPower = -0.15;
        }
        else if (Math.abs(manualVelocityTicksPerSec) >= Math.abs(shooterGoal)) {
            rawPower = calculatePower(shooterGoal);
        }
        double compensatedPower = (rawPower >= 1) ? 1 : rawPower * (voltageCalibration / batteryVoltage);

        compensatedPower = Math.max(-1.0, Math.min(1.0, compensatedPower));
        telemetryM.addData("shooterCompensatedPower", compensatedPower);

        if (state == State.STOPPED) {
            motor1.setPower(0);
        } else {
            motor1.setPower(compensatedPower);
        }

        telemetryM.addData("shooter1 ticks/s", manualVelocityTicksPerSec);
        telemetryM.addData("shooter1 rpm", ticksToRPM(manualVelocityTicksPerSec, 28));
        telemetryM.addData("state", state);
        telemetryM.addData("mode", mode);

        telemetryM.addData("shooterTargetVelo", controlSystem.getGoal().getVelocity());
        telemetryM.addData("shooterCurrentVelo", Math.abs(manualVelocityTicksPerSec));
        telemetryM.addData("shooterVeloOffset", Math.abs(shooterGoal) - Math.abs(manualVelocityTicksPerSec));
    }

    public static final Shooter INSTANCE = new Shooter();
}