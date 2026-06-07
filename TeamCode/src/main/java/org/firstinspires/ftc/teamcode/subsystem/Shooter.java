package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.subsystem.BatteryVars.batteryVoltage;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import com.pedropathing.ivy.Command;

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
    private TelemetryManager telemetryM;

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

    private final ControlSystem controlSystem = ControlSystem.builder()
            .velPid(pidCoefficients)
            .build();

    public Command spinUp = Commands.instant(() -> {
        state = State.RUNNING;
    });

    public Command spinDown = Commands.instant(() -> {
        state = State.STOPPED;
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
        if (state == State.STOPPED) return true;
        // Use manual velocity calculations
        double speed = Math.abs(manualVelocityTicksPerSec);
        double target = Math.abs(controlSystem.getGoal().getVelocity());

        ActiveOpMode.telemetry().addData("SHOOTERSPEED speed", speed);
        ActiveOpMode.telemetry().addData("SHOOTERTARGET target", target);

        return speed >= 1120;
    }

    public double calculateHood2(double x) {
        double a = -1.020e-06;
        double b = 9.253e-04;
        double c = -2.979e-01;
        double d = 8.410e+01;

        return ((a * x + b) * x + c) * x + d;
    }

    public double calculateRPM2(double x) {
        double m = 5.880e+00;
        double b = 1.955e+03;
        return m*x+b;
    }

    public double calculateHood4(double x) {
        double a = -5.303e-06;
        double b = 1.334e-02;
        double c = -7.790e+00;
        double y = a*(x*x)+(b*x)+(c);
        return Math.max(0.5, Math.min(y, 0.68));
    }

    public double calculateHood(double x) {
        double a = 1.044e-05;
        double b = -2.713e-02;
        double c = 1.807e+01;
        double y = a*(x*x)+(b*x)+(c);
        return Math.max(0.45, Math.min(y, 0.63));
    }

    public double calculateRPM(double x) {
        double m = 1.244e+00;
        double b = 1.108e+03;
        return m*x+b;
    }

    @Override
    public void initialize() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        voltageSensor = ActiveOpMode.hardwareMap().get(VoltageSensor.class, "Control Hub");

        hoodServo1.getServo().setDirection(Servo.Direction.FORWARD);
        PwmControl hoodServo1PWM = (PwmControl) hoodServo1.getServo();
        hoodServo1PWM.setPwmRange(new PwmControl.PwmRange(500, 2500, 10000));

        // Seed initial values to prevent a massive spike on the first periodic execution loop
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

        // Cache data for the next periodic cycle
        lastTicks = currentTicks;
        lastTimeNanos = currentTimeNanos;
        // ------------------------------------------

        batteryVoltage = voltageSensor.getVoltage();
        telemetryM.addData("batteryVoltage", batteryVoltage);

        if(true) {
            double distanceHorizontalCm = Webcam.INSTANCE.lastDistanceComponent.horizontal;
            double distanceVerticalCm = Webcam.INSTANCE.lastDistanceComponent.vertical;
            ActiveOpMode.telemetry().addData("shooterDistanceX", distanceHorizontalCm);
            ActiveOpMode.telemetry().addData("shooterDistanceY", distanceVerticalCm);
            double calculatedRPM = calculateRPM(distanceHorizontalCm);
            double calculatedTicks = rpmToTicks(calculatedRPM, 28);

            // Replaced motor1.getVelocity() with manual calculation
            double calculatedHood = calculateHood4(manualVelocityTicksPerSec);

            telemetryM.addData("shooterTicksEstimate", calculatedTicks);
            telemetryM.addData("shooterHoodEstimate", calculatedHood);

            shooterAngle = calculatedHood;
            shooterGoal = calculatedRPM;
            if (mode == Mode.MANUAL) {
                shooterGoal = overrideVelo;
                //shooterAngle = overrideAngle;
            }
            controlSystem.setGoal(new KineticState(Double.MAX_VALUE, shooterGoal, Double.MAX_VALUE));
        }
        hoodServo1.setPosition(shooterAngle);

        double rawPower = 1;
        // Replaced motor1.getVelocity() with manual calculation
        telemetryM.addData("IMPORTANT DATA", Math.abs(shooterGoal) - Math.abs(manualVelocityTicksPerSec));

        if(Math.abs(shooterGoal) < Math.abs(manualVelocityTicksPerSec)) {
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

        // Updated all telemetries to use manual velocity tracking values
        ActiveOpMode.telemetry().addData("shooter1 ticks/s", manualVelocityTicksPerSec);
        ActiveOpMode.telemetry().addData("shooter1 rpm", ticksToRPM(manualVelocityTicksPerSec, 28));
        ActiveOpMode.telemetry().addData("cs power", rawPower);
        ActiveOpMode.telemetry().addData("cs goal", controlSystem.getGoal());
        ActiveOpMode.telemetry().addData("state", state);
        ActiveOpMode.telemetry().addData("mode", mode);

        telemetryM.addData("shooterTargetVelo", controlSystem.getGoal().getVelocity());
        telemetryM.addData("shooterCurrentVelo", Math.abs(manualVelocityTicksPerSec));
    }

    public static final Shooter INSTANCE = new Shooter();
}