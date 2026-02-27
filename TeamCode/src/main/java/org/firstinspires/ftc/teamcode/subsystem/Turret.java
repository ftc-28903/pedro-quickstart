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
    public static double kF = 0.1;

    @Override
    public void initialize() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void periodic() {
        if (Webcam.INSTANCE.detectionTimer.milliseconds() > 800) {
            // do manual control
            servo1.setPower(0);
            servo2.setPower(0);
            return;
        }
        controlSystem.setGoal(new KineticState(0, Double.MAX_VALUE, Double.MAX_VALUE));
        double power = kP * -Webcam.INSTANCE.lastOffset;
        servo1.setPower(power);
        servo2.setPower(power);

        telemetryM.addData("turret power", power);
        telemetryM.addData("offset", Webcam.INSTANCE.lastOffset);
        telemetryM.addData("detectionTimer", Webcam.INSTANCE.detectionTimer.milliseconds());
    }

    public static final Turret INSTANCE = new Turret();
}
