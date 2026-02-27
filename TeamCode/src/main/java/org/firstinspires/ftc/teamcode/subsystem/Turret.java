package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.CRServoEx;

@Configurable
public class Turret implements Subsystem {
    private Turret() { }
    public final CRServoEx servo1 = new CRServoEx("turret1");
    public final CRServoEx servo2 = new CRServoEx("turret2");
    private TelemetryManager telemetryM;

    // --- Prediction params (tune these) ---
    // simple constant-velocity prediction
    public static double predictionLatencySeconds = 0.12;
    public static double velSmoothingAlpha = 0.25;
    public static double maxPrediction = 5.0; // clamp predicted offset (units of lastOffset)

    // --- PID gains used with predicted error ---
    public static double kP = 0.02;
    public static double kI = 0.0;
    public static double kD = 0.001;
    public static double kF = 0.0;

    // --- Loss / out-of-range handling ---
    // how long without a detection before declaring "lost"
    public static long detectionTimeoutMs = 800;
    // if predicted offset magnitude exceeds this, consider it out-of-range/unreachable
    public static double outOfRangeThreshold = 40.0;
    // behaviour when out-of-range: "stop", "nudge", or "sweep"
    // "stop"  -> hold position (power = 0)
    // "nudge" -> apply small power toward the sign of predicted error (try to move closer)
    // "sweep" -> enter sweep search mode (oscillating motion)
    public static String outOfRangeBehavior = "stop";

    // --- Sweep/search mode (used when lost or out-of-range with sweep chosen) ---
    public static boolean enableSearchOnLost = true;
    public static double sweepFrequencyHz = 0.15;    // Hz (how many back-and-forths per second)
    public static double sweepAmplitudePower = 0.45; // power amplitude for CRServo (0..1)

    // internal state
    private double velEstimate = 0.0;
    private double prevMeasuredError = 0.0;
    private double prevPredictedError = 0.0;
    private double integral = 0.0;
    private long lastTime = System.nanoTime();
    private long lostStartTime = -1;

    @Override
    public void initialize() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void periodic() {
        long nowNano = System.nanoTime();
        double dt = (nowNano - lastTime) / 1e9;
        if (dt <= 0) dt = 1e-6;
        lastTime = nowNano;

        // If tag hasn't been detected recently -> lost
        boolean lost = Webcam.INSTANCE.detectionTimer.milliseconds() > detectionTimeoutMs;
        if (lost) {
            if (lostStartTime < 0) lostStartTime = System.currentTimeMillis();
        } else {
            lostStartTime = -1;
        }

        if (lost) {
            // lost: determine whether to search or hold
            if (enableSearchOnLost && "sweep".equalsIgnoreCase(outOfRangeBehavior)) {
                double nowSec = System.currentTimeMillis() / 1000.0;
                // sinusoidal sweep control (back-and-forth)
                double sweep = Math.sin(2.0 * Math.PI * sweepFrequencyHz * nowSec) * sweepAmplitudePower;
                servo1.setPower(sweep);
                servo2.setPower(sweep);
                telemetryM.addData("turret mode", "lost - sweeping");
                telemetryM.addData("sweep power", sweep);
            } else if (enableSearchOnLost && "nudge".equalsIgnoreCase(outOfRangeBehavior)) {
                // small constant nudge to attempt reacquire (direction unknown -> small oscillation)
                double nud = 0.12 * Math.sin(System.currentTimeMillis() / 1000.0 * 2.0 * Math.PI * 0.5);
                servo1.setPower(nud);
                servo2.setPower(nud);
                telemetryM.addData("turret mode", "lost - nudging");
                telemetryM.addData("nudge power", nud);
            } else {
                // default: stop motors to avoid wasted motion
                servo1.setPower(0);
                servo2.setPower(0);
                telemetryM.addData("turret mode", "lost - stopped");
            }
            return;
        }

        // Tag is visible: compute predicted error and use PID
        double measuredError = Webcam.INSTANCE.lastOffset;

        // velocity estimation (instantaneous then smoothed)
        double measuredVel = (measuredError - prevMeasuredError) / dt;
        velEstimate = (1.0 - velSmoothingAlpha) * velEstimate + velSmoothingAlpha * measuredVel;

        // prediction
        double predictedError = measuredError + velEstimate * predictionLatencySeconds;
        // clamp prediction magnitude
        if (predictedError > maxPrediction) predictedError = maxPrediction;
        if (predictedError < -maxPrediction) predictedError = -maxPrediction;

        // check out-of-range
        boolean outOfRange = Math.abs(predictedError) > outOfRangeThreshold;

        if (outOfRange) {
            // handle per configured behavior
            if ("stop".equalsIgnoreCase(outOfRangeBehavior)) {
                servo1.setPower(0);
                servo2.setPower(0);
                telemetryM.addData("turret mode", "out-of-range - stopped");
                telemetryM.addData("predicted error (clamped)", predictedError);
                // still update state and exit
                prevMeasuredError = measuredError;
                prevPredictedError = predictedError;
                return;
            } else if ("nudge".equalsIgnoreCase(outOfRangeBehavior)) {
                // small power toward the sign of predictedError
                double nudgePower = 0.25 * Math.signum(predictedError);
                servo1.setPower(nudgePower);
                servo2.setPower(nudgePower);
                telemetryM.addData("turret mode", "out-of-range - nudging");
                telemetryM.addData("nudge power", nudgePower);
                prevMeasuredError = measuredError;
                prevPredictedError = predictedError;
                return;
            } else if ("sweep".equalsIgnoreCase(outOfRangeBehavior)) {
                // fallthrough to sweeping behavior (same sweeping as lost)
                double nowSec = System.currentTimeMillis() / 1000.0;
                double sweep = Math.sin(2.0 * Math.PI * sweepFrequencyHz * nowSec) * sweepAmplitudePower;
                servo1.setPower(sweep);
                servo2.setPower(sweep);
                telemetryM.addData("turret mode", "out-of-range - sweeping");
                telemetryM.addData("sweep power", sweep);
                prevMeasuredError = measuredError;
                prevPredictedError = predictedError;
                return;
            } else {
                // unknown behavior string -> safe default: stop
                servo1.setPower(0);
                servo2.setPower(0);
                telemetryM.addData("turret mode", "out-of-range - unknown behavior -> stopped");
                prevMeasuredError = measuredError;
                prevPredictedError = predictedError;
                return;
            }
        }

        // Normal reachable case: PID on predicted error
        integral += predictedError * dt;
        double derivative = (predictedError - prevPredictedError) / dt;
        prevPredictedError = predictedError;

        double pid = (kP * predictedError) + (kI * integral) + (kD * derivative);
        double feedforward = kF * Math.signum(predictedError);
        double power = pid + feedforward;

        // clamp to [-1,1]
        power = Math.max(-1.0, Math.min(1.0, power));

        servo1.setPower(power);
        servo2.setPower(power);

        // telemetry
        telemetryM.addData("turret mode", "tracking");
        telemetryM.addData("power", power);
        telemetryM.addData("measured error", measuredError);
        telemetryM.addData("predicted error", predictedError);
        telemetryM.addData("vel estimate", velEstimate);
        telemetryM.addData("outOfRangeThreshold", outOfRangeThreshold);

        // update previous measured
        prevMeasuredError = measuredError;
    }

    public static final Turret INSTANCE = new Turret();
}