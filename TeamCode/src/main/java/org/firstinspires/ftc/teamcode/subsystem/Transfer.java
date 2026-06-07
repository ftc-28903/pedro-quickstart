package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

import com.pedropathing.ivy.Command;

@Configurable
public class Transfer implements Subsystem {
    public static final Transfer INSTANCE = new Transfer();
    public static double detectDist = 35;
    public static double maxMotorSpeed = 0.6;
    // TODO: CHANGE THIS for flywheel
    public static double maxOverrideSpeed = 1.0;
    public static double readDelay = 0;
    public static double blockerDelayMs = 200;

    // Pulse timing configurations
    public static double pulseRunTimeMs = 500;
    public static double pulsePauseTimeMs = 150;

    private Transfer() {}

    public ElapsedTime colorGetTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public ElapsedTime blockerOpenTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public ElapsedTime pulseTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS); // Timer to track the 650ms cycle

    public enum State {
        AUTO,
        OVERRIDE,
        OFF_OVERRIDE
    }

    public State state = State.AUTO;
    public double lastDistance = 0.0;
    private boolean wasBlockerClosed = true;
    private boolean wasTransferRunning = false; // Tracks if transfer was active in the last loop to reset pulse timer

    public final MotorEx motor1 = new MotorEx("intake2").reversed();
    private final ServoEx blockerServo = new ServoEx("blocker");
    public RevColorSensorV3 colorSensorV3;
    private TelemetryManager telemetryM;

    public ElapsedTime overrideCycleTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    // Refactored commands using instant()
    public Command overrideOn = Commands.instant(() -> {
        state = State.OVERRIDE;
        overrideCycleTimer.reset();
    });
    public Command spinUpReverse = Commands.instant(() -> motor1.setPower(-1));

    public Command overrideOff = Commands.instant(() -> {
        if (state == State.OVERRIDE) {
            state = State.AUTO;
        }
    });

    public Command offOverrideOn = Commands.instant(() -> state = State.OFF_OVERRIDE);
    public Command offOverrideOff = Commands.instant(() -> {
        if (state == State.OFF_OVERRIDE) {
            state = State.AUTO;
        }
    });

    @Override
    public void initialize() {
        colorSensorV3 = ActiveOpMode.hardwareMap().get(RevColorSensorV3.class, "color_sensor");
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        blockerServo.getServo().setDirection(Servo.Direction.FORWARD);
        PwmControl blockerServoPWM = (PwmControl) blockerServo.getServo();
        blockerServoPWM.setPwmRange(new PwmControl.PwmRange(1550, 1850, 10000));
        blockerOpenTimer.reset();
        pulseTimer.reset();
    }

    /**
     * Determines if the motor should currently be paused based on the pulse cycle.
     */
    private boolean shouldPulsePause() {
        double currentCycleTime = pulseTimer.milliseconds();
        double totalCycleTime = pulseRunTimeMs + pulsePauseTimeMs;

        // Reset the timer automatically once a full cycle finishes
        if (currentCycleTime >= totalCycleTime) {
            pulseTimer.reset();
            return false;
        }

        // If we have surpassed the run time, we are in the pause window
        return currentCycleTime >= pulseRunTimeMs;
    }

    @Override
    public void periodic() {
        if (state == State.AUTO) {
            if (colorGetTimer.milliseconds() > readDelay) {
                lastDistance = colorSensorV3.getDistance(DistanceUnit.MM);
                colorGetTimer.reset();
            }
        }

        telemetryM.addData("lastDistance", lastDistance);
        ActiveOpMode.telemetry().addData("transfer power", motor1.getPower());
        ActiveOpMode.telemetry().addData("is speed good", Shooter.INSTANCE.isSpeedGood());
        ActiveOpMode.telemetry().addData("intake2 amp", motor1.getMotor().getCurrent(CurrentUnit.MILLIAMPS));
        ActiveOpMode.telemetry().addData("blocker position", blockerServo.getPosition());
        ActiveOpMode.telemetry().addData("transfer state", state);

        if (state == State.OFF_OVERRIDE) {
            blockerServo.setPosition(0);
            motor1.setPower(0);
            wasBlockerClosed = true;
            wasTransferRunning = false;
            return;
        }

        // Check if the system actually wants to run the transfer
        boolean wantsToRun = (state == State.OVERRIDE) || (lastDistance > detectDist);

        if (wantsToRun) {
            // If the transfer just started running this loop, reset the pulse timer so it always starts by running
            if (!wasTransferRunning) {
                pulseTimer.reset();
                wasTransferRunning = true;
            }
        } else {
            wasTransferRunning = false;
        }

        // --- OVERRIDE LOGIC ---
        if (state == State.OVERRIDE) {
            if (wasBlockerClosed) {
                blockerOpenTimer.reset();
                wasBlockerClosed = false;
            }

            blockerServo.setPosition(1);

            if (blockerOpenTimer.milliseconds() >= blockerDelayMs && Shooter.INSTANCE.isSpeedGood()) {
                // Apply the pulse restriction here
                if (shouldPulsePause()) {
                    motor1.setPower(0);
                } else {
                    motor1.setPower(maxOverrideSpeed);
                }
            } else {
                motor1.setPower(0);
            }
            return;
        }

        // --- REGULAR TRANSFER LOGIC (DISTANCE BASED) ---
        if (lastDistance > detectDist) {
            if (wasBlockerClosed) {
                blockerOpenTimer.reset();
                wasBlockerClosed = false;
            }

            blockerServo.setPosition(0);

            if (blockerOpenTimer.milliseconds() >= blockerDelayMs) {
                // Apply the pulse restriction here
                if (shouldPulsePause()) {
                    motor1.setPower(0);
                } else {
                    motor1.setPower(maxMotorSpeed);
                }
            } else {
                motor1.setPower(0);
            }
            return;
        }

        // --- IDLE STATE ---
        blockerServo.setPosition(0);
        motor1.setPower(0);
        wasBlockerClosed = true;
    }
}