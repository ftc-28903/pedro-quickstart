package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.teamcode.utils.ShooterCalculator;
import org.firstinspires.ftc.teamcode.utils.ShotPoint;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

@Configurable
public class Shooter implements Subsystem {
    public boolean shouldStop = true;
    private Shooter() { }

    public final MotorEx motor1 = new MotorEx("shooter1").reversed();
    public final MotorEx motor2 = new MotorEx("shooter2");
    private final ServoEx servo1 = new ServoEx("hood1");
    private TelemetryManager telemetryM;

    public static double shooterGoal = 1350;
    public static double shooterAngle = 0.5;
    public static BasicFeedforwardParameters feedforwardParameters = new BasicFeedforwardParameters(0.000475, 0.0, 0.0);
    public static PIDCoefficients pidCoefficients = new PIDCoefficients(0.00002, 0, 0.0);
    public static double velocityTolerance = 50;

    private final ControlSystem controlSystem = ControlSystem.builder()
            .basicFF(feedforwardParameters)
            .velSquID(pidCoefficients)
            .build();

    public Command spinUp = new InstantCommand(() -> {
        shouldStop = false;
        //motor.setPower(1);
    });

    public Command spinDown = new InstantCommand(() -> {
        shouldStop = true;
        //motor.setPower(0);
    });

    // ticksPerSecond = RPM x 28 / 60
    public double ticksToRPM(double ticksPerSecond, double countsPerRevolution) {
        return (ticksPerSecond / countsPerRevolution * 60);
    }

    public boolean isSpeedGood() {
        if (shouldStop) return true;
        double speed = -motor1.getVelocity();
        double target = controlSystem.getGoal().getVelocity();
        
        return speed >= target - velocityTolerance;
    }

    @Override
    public void initialize() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void periodic() {
        if(!shouldStop) {
            double distanceCm = Webcam.INSTANCE.lastDistanceComponent.horizontal;
            ShotPoint calculatedShot = ShooterCalculator.calculateShot(distanceCm);
            ActiveOpMode.telemetry().addData("shooterDistance", calculatedShot.distanceCm);
            //shooterAngle = calculatedShot.hood;
            //shooterGoal = calculatedShot.velocity;
            controlSystem.setGoal(new KineticState(Double.MAX_VALUE, shooterGoal, Double.MAX_VALUE));
        }
        servo1.setPosition(shooterAngle);

        double power = controlSystem.calculate(motor1.getState());
        if (shouldStop) {
            motor1.setPower(0);
            motor2.setPower(0);
        } else {
            motor1.setPower(power);
            motor2.setPower(power);
        }
        ActiveOpMode.telemetry().addData("shooter1 ticks/s", motor1.getVelocity());
        ActiveOpMode.telemetry().addData("shooter1 rpm", ticksToRPM(motor1.getVelocity(), 28));
        ActiveOpMode.telemetry().addData("shooter2 ticks/s", motor2.getVelocity());
        ActiveOpMode.telemetry().addData("shooter2 rpm", ticksToRPM(motor2.getVelocity(), 28));
        ActiveOpMode.telemetry().addData("cs power", power);
        ActiveOpMode.telemetry().addData("cs goal", controlSystem.getGoal());
        ActiveOpMode.telemetry().addData("shouldStop", shouldStop);

        telemetryM.addData("shooterTargetVelo", controlSystem.getGoal().getVelocity());
        telemetryM.addData("shooterCurrentVelo", -motor1.getVelocity());
    }

    public static final Shooter INSTANCE = new Shooter();
}
