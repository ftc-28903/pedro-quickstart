package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.subsystem.BatteryVars.batteryVoltage;

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

    public static double kP = 0.017;
    public static double kF = 0.035;

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

    public boolean seesTag = false;

    public Command disableTurret = new InstantCommand(() -> {
        manualOverride = true;
    });

    public Command enableTurret = new InstantCommand(() -> {
        manualOverride = false;
    });

    public Command waitForTurret = new WaitUntil(() -> seesTag);
    public boolean isTurretInRange() {
        return Math.abs(Webcam.INSTANCE.lastOffset) < 5;
    }

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

        seesTag = Webcam.INSTANCE.detectionTimer.milliseconds() <= 300;

        if (seesTag) {
            // --- Update velocity estimate ---
            double dt = now - lastTimestamp;
            if (dt > 0) {
                offsetVelocity = (Webcam.INSTANCE.lastOffset - lastOffset) / dt;
            }

            lastOffset = Webcam.INSTANCE.lastOffset;
            lastTimestamp = now;
            tagLostTime = -1;

            double power = (kP * -Webcam.INSTANCE.lastOffset) * (13/batteryVoltage);
            power += Math.signum(power) * kF;
            power = Math.max(-0.5, Math.min(0.5, power));

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
                // --- Full power nudge in last known velocity direction ---
                double nudgeDirection = Math.signum(offsetVelocity);
                if (nudgeDirection == 0) nudgeDirection = Math.signum(lastOffset);

                double power = -nudgeDirection;
                power = Math.max(-0.75, Math.min(0.75, power));

                servo1.setPower(power);
                servo2.setPower(power);

                telemetryM.addData("mode", "NUDGING");
                telemetryM.addData("nudgeDirection", nudgeDirection);
            }
            else {
                // --- SCAN MODE — sweep toward last known offset side ---
                double scanTime = now - tagLostTime - 1.0;

                double direction = Math.signum(lastOffset);
                if (direction == 0) direction = 1;

                double power = direction * Math.sin(2 * Math.PI * scanFrequency * scanTime);
                power = Math.max(-0.75, Math.min(0.75, power));

                servo1.setPower(power);
                servo2.setPower(power);

                telemetryM.addData("mode", "SCANNING");
                telemetryM.addData("scanToward", direction);
                telemetryM.addData("scanPower", power);
            }
        }
    }

    public static final Turret INSTANCE = new Turret();
}
