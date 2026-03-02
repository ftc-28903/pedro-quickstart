package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.subsystem.ff.ShooterFeedforward;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;
import dev.nextftc.control.feedforward.FeedforwardElement;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.powerable.PowerableGroup;

@Configurable
public class Turret implements Subsystem {
    public boolean shouldStop = true;
    private Turret() { }
    public final CRServoEx servo1 = new CRServoEx("turret1");
    public final CRServoEx servo2 = new CRServoEx("turret2");
    private TelemetryManager telemetryM;
    public static PIDCoefficients pidCoefficients = new PIDCoefficients(2, 0, 0.0);
    public static BasicFeedforwardParameters feedforwardParameters = new BasicFeedforwardParameters(0,0,0);

    private final ControlSystem controlSystem = ControlSystem.builder()
            .velSquID(pidCoefficients)
            .basicFF(feedforwardParameters)
            .build();

    public static double kP = 0.02;
    public static double kF = 0.025;

    // Prediction state
    private double lastOffset = 0;
    private double lastTimestamp = 0;
    private double offsetVelocity = 0; // units per second
    private double tagLostTime = -1;

    public static boolean manualOverride = false;
    public static double overrideSpeed = 0;

    // Scan behavior
    public static double scanAmplitude = 2;   // max servo power
    public static double scanFrequency = 0.3;   // cycles per second

    @Override
    public void initialize() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void periodic() {
        if (manualOverride) {
            servo1.setPower(overrideSpeed);
            servo2.setPower(overrideSpeed);
            return;
        }

        double now = System.currentTimeMillis() / 1000.0;

        boolean seesTag = Webcam.INSTANCE.detectionTimer.milliseconds() <= 800;

        if (seesTag) {
            // --- Update velocity estimate ---
            double dt = now - lastTimestamp;
            if (dt > 0) {
                offsetVelocity = (Webcam.INSTANCE.lastOffset - lastOffset) / dt;
            }

            lastOffset = Webcam.INSTANCE.lastOffset;
            lastTimestamp = now;
            tagLostTime = -1;

            double power = kP * -Webcam.INSTANCE.lastOffset;
            if (power < 0) {
                power -= kF;
            } else {
                power += kF;
            }

            servo1.setPower(power);
            servo2.setPower(power);

            telemetryM.addData("mode", "TRACKING");
            telemetryM.addData("offset", lastOffset);
            telemetryM.addData("velocity", offsetVelocity);
            telemetryM.addData("power", power);
        }
        else {
            // --- Tag just got lost ---
            if (tagLostTime < 0) {
                tagLostTime = now;
            }

            double timeSinceLost = now - tagLostTime;

            if (timeSinceLost <= 1) {
                // --- Predict offset ---
                double predictedOffset = lastOffset + offsetVelocity * timeSinceLost;

                //double power = kP * -predictedOffset;
                double power = 1;

                servo1.setPower(power);
                servo2.setPower(power);

                telemetryM.addData("mode", "PREDICTING");
                telemetryM.addData("predictedOffset", predictedOffset);
                telemetryM.addData("velocity", offsetVelocity);
                telemetryM.addData("power", power);
            }
            else {
                // --- SCAN MODE (back and forth) ---
                double scanTime = now - tagLostTime - 1.0; // start scanning after 1s

                double power = scanAmplitude * Math.sin(2 * Math.PI * scanFrequency * scanTime);

                if (power < 0) {
                    power *= 1.2;
                }

                servo1.setPower(power);
                servo2.setPower(power);

                telemetryM.addData("mode", "SCANNING");
                telemetryM.addData("scanPower", power);
            }
        }
    }

    public static final Turret INSTANCE = new Turret();
}
