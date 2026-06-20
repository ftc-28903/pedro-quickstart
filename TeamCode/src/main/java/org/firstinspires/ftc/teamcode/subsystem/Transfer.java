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
import org.firstinspires.ftc.teamcode.utils.AutoStorage;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

import com.pedropathing.ivy.Command;

@Configurable
public class Transfer implements Subsystem {
    public static final Transfer INSTANCE = new Transfer();
    public static double detectDist = 80;
    public static double maxMotorSpeed = 0.5;

    // TODO: CHANGE THIS for flywheel
    public static double maxOverrideSpeed = 1.0;
    public static double readDelay = 0;
    public static double blockerDelayMs = 350;

    // Pulse timing configurations
    public static double pulseRunTimeMs = 500;
    public static double pulsePauseTimeMs = 0;

    // --- Ball Anti-Jam/Reverse Tuning Variables ---
    public static double ballDetectionThresholdMs = 3000; // 3 seconds
    public static double reverseDurationMs = 0;          // 150ms
    public static double reverseSpeed = 0;              // Speed when backing up
    public static double maxOverrideReverseSpeed = -1.0;   // Speed for manual force push backwards

    public static double blocker_block_pos = 0.4;

    private Transfer() {}

    public ElapsedTime colorGetTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public ElapsedTime blockerOpenTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public ElapsedTime pulseTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    // --- Timers and flags for the anti-jam logic ---
    public ElapsedTime ballDetectionTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private boolean isDetectingBall = false;
    private boolean hasReversedForBall = false;

    // Driver 1 States
    public enum State {
        AUTO,
        OVERRIDE,
        OFF_OVERRIDE
    }

    // --- SEPARATED: Driver 2 States ---
    public enum ForceState {
        NONE,
        FORCE_PUSH,
        FORCE_PUSH_BACK
    }

    public State state = State.AUTO;
    public ForceState forceState = ForceState.NONE;

    public double lastDistance = 0.0;
    private boolean wasBlockerClosed = true;
    private boolean wasTransferRunning = false;

    public final MotorEx motor1 = new MotorEx("intake2").reversed();
    private final ServoEx blockerServo = new ServoEx("blocker");
    public RevColorSensorV3 colorSensorV3;

    public ElapsedTime overrideCycleTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private TelemetryManager telemetryM;

    // --- DRIVER 1 COMMANDS ---
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

    // --- REFACTORED DRIVER 2 COMMANDS ---
    public Command forcePush = Commands.instant(() -> forceState = ForceState.FORCE_PUSH);
    public Command forcePushStop = Commands.instant(() -> {
        if (forceState == ForceState.FORCE_PUSH) {
            forceState = ForceState.NONE;
        }
    });

    public Command forcePushBack = Commands.instant(() -> forceState = ForceState.FORCE_PUSH_BACK);
    public Command forcePushBackStop = Commands.instant(() -> {
        if (forceState == ForceState.FORCE_PUSH_BACK) {
            forceState = ForceState.NONE;
        }
    });

    @Override
    public void initialize() {
        blockerServo.getServo().setPosition(1);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        colorSensorV3 = ActiveOpMode.hardwareMap().get(RevColorSensorV3.class, "color_sensor");

        blockerServo.getServo().setDirection(Servo.Direction.FORWARD);
        PwmControl blockerServoPWM = (PwmControl) blockerServo.getServo();
        blockerServoPWM.setPwmRange(new PwmControl.PwmRange(1250, 1900, 10000));
        blockerOpenTimer.reset();
        pulseTimer.reset();
        ballDetectionTimer.reset();
        blockerServo.getServo().setPosition(blocker_block_pos);
    }

    private boolean shouldPulsePause() {
        double currentCycleTime = pulseTimer.milliseconds();
        double totalCycleTime = pulseRunTimeMs + pulsePauseTimeMs;

        if (currentCycleTime >= totalCycleTime) {
            pulseTimer.reset();
            return false;
        }

        return currentCycleTime >= pulseRunTimeMs;
    }

    @Override
    public void periodic() {
        if (!AutoStorage.opModeStarted) return;
        if (state == State.AUTO) {
            if (colorGetTimer.milliseconds() > readDelay) {
                lastDistance = colorSensorV3.getDistance(DistanceUnit.MM);
                colorGetTimer.reset();
            }
        }

        telemetryM.addData("lastDistance", lastDistance);
        telemetryM.addData("transfer power", motor1.getPower());
        telemetryM.addData("is speed good", Shooter.INSTANCE.isSpeedGood());
        telemetryM.addData("intake2 amp", motor1.getMotor().getCurrent(CurrentUnit.MILLIAMPS));
        telemetryM.addData("blocker position", blockerServo.getServo().getPosition());
        telemetryM.addData("transfer state", state);
        telemetryM.addData("force state", forceState);

        // ====================================================================
        // PRIORITY 1: DRIVER 2 FORCE OVERRIDES (Respects Driver 1 Servo Position)
        // ====================================================================
        if (forceState == ForceState.FORCE_PUSH) {
            if (state == State.OVERRIDE) {
                blockerServo.getServo().setPosition(1);
            } else {
                blockerServo.getServo().setPosition(blocker_block_pos);
            }

            motor1.setPower(maxOverrideSpeed);
            wasBlockerClosed = false;
            return;
        }

        if (forceState == ForceState.FORCE_PUSH_BACK) {
            if (state == State.OVERRIDE) {
                blockerServo.getServo().setPosition(1);
            } else {
                blockerServo.getServo().setPosition(blocker_block_pos);
            }

            motor1.setPower(maxOverrideReverseSpeed);
            wasBlockerClosed = false;
            return;
        }

        // ====================================================================
        // PRIORITY 2: DRIVER 1 STANDARD STATE MACHINE
        // ====================================================================
        if (state == State.OFF_OVERRIDE) {
            blockerServo.getServo().setPosition(blocker_block_pos);
            motor1.setPower(0);
            wasBlockerClosed = true;
            wasTransferRunning = false;
            isDetectingBall = false;
            return;
        }

        // --- Track how long the ball has been in front of the sensor ---
        boolean currentBallDetected = (lastDistance > detectDist) && (state == State.AUTO);

        if (currentBallDetected) {
            if (!isDetectingBall) {
                ballDetectionTimer.reset();
                isDetectingBall = true;
            }
        } else {
            isDetectingBall = false;
            hasReversedForBall = false;
        }

        // --- Check if we need to actively execute the reverse pulse ---
        boolean shiftingBackwards = false;
        if (isDetectingBall && ballDetectionTimer.milliseconds() >= ballDetectionThresholdMs) {
            double timeSinceThreshold = ballDetectionTimer.milliseconds() - ballDetectionThresholdMs;

            if (timeSinceThreshold < reverseDurationMs) {
                shiftingBackwards = true;
                hasReversedForBall = true;
            }
        }

        if (shiftingBackwards) {
            blockerServo.getServo().setPosition(blocker_block_pos);
            motor1.setPower(reverseSpeed);
            return;
        }

        // Check if the system actually wants to run the transfer
        boolean wantsToRun = (state == State.OVERRIDE) || (lastDistance > detectDist);

        if (wantsToRun) {
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

            blockerServo.getServo().setPosition(1);

            if (blockerOpenTimer.milliseconds() >= blockerDelayMs && Shooter.INSTANCE.isSpeedGood()) {
                if (shouldPulsePause()) {
                    motor1.setPower(blocker_block_pos);
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

            blockerServo.getServo().setPosition(blocker_block_pos);

            if (blockerOpenTimer.milliseconds() >= blockerDelayMs) {
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
        blockerServo.getServo().setPosition(blocker_block_pos);
        motor1.setPower(0);
        wasBlockerClosed = true;
    }
}