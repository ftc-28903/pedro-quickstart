package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.subsystem.ff.ShooterFeedforward;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

@Configurable
public class Shooter implements Subsystem {
    public boolean shouldStop = true;
    private Shooter() { }

    public VoltageSensor voltageSensor;
    public final MotorEx motor1 = new MotorEx("shooter1").reversed();
    public final MotorEx motor2 = new MotorEx("shooter2");
    private final ServoEx servo1 = new ServoEx("hood1");
    private TelemetryManager telemetryM;

    public static double shooterGoal = 1250;
    public static double shooterPowerOverride = 0.1;
    public static double shooterAngle = 0.58;
    public static PIDCoefficients pidCoefficients = new PIDCoefficients(0.008, 0, 0.0);
    public static double velocityTolerance = 100;
    public static double voltageCalibration = 13.0;

    private final ControlSystem controlSystem = ControlSystem.builder()
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
    public Command waitForSpeed = new WaitUntil(this::isSpeedGood);

    // ticksPerSecond = RPM x 28 / 60
    public double ticksToRPM(double ticksPerSecond, double countsPerRevolution) {
        return (ticksPerSecond / countsPerRevolution * 60);
    }
    public double rpmToTicks(double rpm, double countsPerRevolution) {
        return (rpm * countsPerRevolution / 60);
    }

    public boolean isSpeedGood() {
        if (shouldStop) return true;
        double speed = -motor1.getVelocity();
        double target = controlSystem.getGoal().getVelocity();
        
        return speed >= target - velocityTolerance;
    }

    public double calculateHood2(double x) {
        // https://curve.fit/EYtVgRN7/single/20260128111854
        double a = -1.020e-06;
        double b = 9.253e-04;
        double c = -2.979e-01;
        double d = 8.410e+01;

        return ((a * x + b) * x + c) * x + d;
    }

    public double calculateRPM2(double x) {
        // https://curve.fit/zMDoKIss/single/20260128111013
        double m = 5.880e+00;
        double b = 1.955e+03;
        return m*x+b;
    }

    public double calculateHood(double x) {
        if (x > 270) {
            return 0.85;
        }
        return 0.8;
    }

    public double calculateRPM(double x) {
        // https://curve.fit/KjezG82L/single/20260130071921
        double m = 1.402e+00;
        double b = 1.032e+03;

        double result = m*x+b;
        if (x > 270) {
            return result+20;
        } else {
            return result;
        }
    }

    @Override
    public void initialize() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        voltageSensor = ActiveOpMode.hardwareMap().get(VoltageSensor.class, "Control Hub");
    }

    private static final double a = 2.824e-10;
    private static final double b = -7.497e-07;
    private static final double c = 1.013e-03;
    private static final double d = -7.218e-02;

    public double calculate(double velo) {
        double speed = velo + 80; // Your +80 offset
        return ((a * speed + b) * speed + c) * speed + d;
    }

    @Override
    public void periodic() {
        double batteryVoltage = voltageSensor.getVoltage();
        telemetryM.addData("batteryVoltage", batteryVoltage);

        if(true) {
            double distanceHorizontalCm = Webcam.INSTANCE.lastDistanceComponent.horizontal;
            double distanceVerticalCm = Webcam.INSTANCE.lastDistanceComponent.vertical;
            ActiveOpMode.telemetry().addData("shooterDistanceX", distanceHorizontalCm);
            ActiveOpMode.telemetry().addData("shooterDistanceY", distanceVerticalCm);
            double calculatedRPM = calculateRPM(distanceHorizontalCm);
            double calculatedTicks = rpmToTicks(calculatedRPM, 28);
            double calculatedHood = calculateHood(distanceHorizontalCm);

            telemetryM.addData("shooterTicksEstimate", calculatedTicks);
            telemetryM.addData("shooterHoodEstimate", calculatedHood);

            shooterAngle = calculatedHood;
            shooterGoal = calculatedRPM;
            controlSystem.setGoal(new KineticState(Double.MAX_VALUE, shooterGoal, Double.MAX_VALUE));
        }
        servo1.setPosition(shooterAngle);

        double rawPower = calculate(shooterGoal) + controlSystem.calculate(new KineticState(Double.MAX_VALUE, shooterGoal, Double.MAX_VALUE));

        double compensatedPower = rawPower * (voltageCalibration / batteryVoltage);

        // Prevent clipping explosions
        compensatedPower = Math.max(-1.0, Math.min(1.0, compensatedPower));
        telemetryM.addData("shooterCompensatedPower", compensatedPower);
        if (shouldStop) {
            motor1.setPower(0);
            motor2.setPower(0);
        } else {
            motor1.setPower(compensatedPower);
            motor2.setPower(compensatedPower);
        }
        ActiveOpMode.telemetry().addData("shooter1 ticks/s", motor1.getVelocity());
        ActiveOpMode.telemetry().addData("shooter1 rpm", ticksToRPM(motor1.getVelocity(), 28));
        ActiveOpMode.telemetry().addData("shooter2 ticks/s", motor2.getVelocity());
        ActiveOpMode.telemetry().addData("shooter2 rpm", ticksToRPM(motor2.getVelocity(), 28));
        ActiveOpMode.telemetry().addData("cs power", rawPower);
        ActiveOpMode.telemetry().addData("cs goal", controlSystem.getGoal());
        ActiveOpMode.telemetry().addData("shouldStop", shouldStop);

        telemetryM.addData("shooterTargetVelo", controlSystem.getGoal().getVelocity());
        telemetryM.addData("shooterCurrentVelo", Math.abs(motor1.getVelocity()));
    }

    public static final Shooter INSTANCE = new Shooter();
}
