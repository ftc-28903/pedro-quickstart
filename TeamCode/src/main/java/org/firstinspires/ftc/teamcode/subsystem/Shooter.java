package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.subsystem.BatteryVars.batteryVoltage;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.VoltageSensor;

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
import org.firstinspires.ftc.teamcode.subsystem.BatteryVars;

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
    public static PIDCoefficients pidCoefficients = new PIDCoefficients(0.00025, 0, 0.0);
    public static double velocityTolerance = 200;
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
        double speed = Math.abs(motor1.getVelocity());
        double target = controlSystem.getGoal().getVelocity();

        ActiveOpMode.telemetry().addData("SHOOTERSPEED speed", speed);
        ActiveOpMode.telemetry().addData("SHOOTERTARGET target", target);
        
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
        /*if (x < 140) {
            return 0.5;
        }*/
        return 0.6;
    }

    public double calculateRPM(double x) {
        // https://curve.fit/KjezG82L/single/20260130071921
        double m = 1.244e+00;
        double b = 1.108e+03;

        /*if (x < 140) {
            return m*(x-30)+b;
        }*/

        return m*x+b;
    }

    @Override
    public void initialize() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        voltageSensor = ActiveOpMode.hardwareMap().get(VoltageSensor.class, "Control Hub");
    }

    public double calculatePower(double velocity) {
        velocity = velocity+30;
        double m = 3.811e-04;
        double b = 1.420e-01;
        return m * velocity + b;
    }

    public static boolean override = false;
    public static double overrideVelo = 0;
    public static double overrideAngle = 0.5;

    @Override
    public void periodic() {
        batteryVoltage = voltageSensor.getVoltage();
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
            if (override) {
                shooterAngle = overrideAngle;
                shooterGoal = overrideVelo;
            }
            controlSystem.setGoal(new KineticState(Double.MAX_VALUE, shooterGoal, Double.MAX_VALUE));
        }
        servo1.setPosition(shooterAngle);

        double rawPower = calculatePower(shooterGoal) + controlSystem.calculate(new KineticState(Double.MAX_VALUE, Math.abs(motor1.getVelocity()), Double.MAX_VALUE));
        rawPower = 1;
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
