package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
public class Shooter implements Subsystem {
    public boolean shouldStop = true;
    private Shooter() { }

    public final MotorEx motor1 = new MotorEx("shooter1").reversed();
    public final MotorEx motor2 = new MotorEx("shooter2");

    public static double shooterGoal = 1550;
    public static BasicFeedforwardParameters feedforwardParameters = new BasicFeedforwardParameters(0.00027, 0.0, 0.0);
    public static PIDCoefficients pidCoefficients = new PIDCoefficients(0.000165, 0, 0.0);

    private final ControlSystem controlSystem = ControlSystem.builder()
            .basicFF(feedforwardParameters)
            .velPid(pidCoefficients)
            .build();

    public Command spinUp = new InstantCommand(() -> {
        controlSystem.setGoal(new KineticState(Double.MAX_VALUE, shooterGoal, Double.MAX_VALUE));
        shouldStop = false;
        //motor.setPower(1);
    });

    public Command spinDown = new InstantCommand(() -> {
        shouldStop = true;
        controlSystem.setGoal(new KineticState(Double.MAX_VALUE, 0, Double.MAX_VALUE));
        //motor.setPower(0);
    });

    // ticksPerSecond = RPM x 28 / 60
    public double ticksToRPM(double ticksPerSecond, double countsPerRevolution) {
        return (ticksPerSecond / countsPerRevolution * 60);
    }

    @Override
    public void periodic() {
        double power = controlSystem.calculate(motor1.getState());

        if (shouldStop) {
            motor1.setPower(0);
            motor2.setPower(0);
        } else {
            controlSystem.setGoal(new KineticState(Double.MAX_VALUE, shooterGoal, Double.MAX_VALUE));
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
    }

    public static final Shooter INSTANCE = new Shooter();
}
