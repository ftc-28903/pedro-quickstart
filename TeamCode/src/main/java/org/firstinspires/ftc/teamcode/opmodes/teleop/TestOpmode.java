package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.Transfer;
import org.firstinspires.ftc.teamcode.subsystem.Webcam;
import org.firstinspires.ftc.teamcode.subsystem.ff.ShooterFeedforward;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.FieldCentric;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name = "TestOpmode")
public class TestOpmode extends NextFTCOpMode {
    public TestOpmode() {
        addComponents(
                BulkReadComponent.INSTANCE
        );
    }

    private final IMUEx imu = new IMUEx("imu", Direction.UP, Direction.FORWARD);

    private final ElapsedTime loopTimeTimer = new ElapsedTime();

    private final MotorEx frontLeftMotor = new MotorEx("front_left");
    private final MotorEx frontRightMotor = new MotorEx("front_right").reversed();
    private final MotorEx backLeftMotor = new MotorEx("back_left");
    private final MotorEx backRightMotor = new MotorEx("back_right").reversed();

    private TelemetryManager telemetryM;

    private final double slowModeStep = 0.25;

    private boolean slowMode = false;
    private double slowModeMultiplier = 0.25;
    private boolean headingLock = false;
    private double headingLockPower = 0.0;

    public static PIDCoefficients headingPIDCoefficients = new PIDCoefficients(0, 0, 0.0);
    private final ControlSystem controlSystem = ControlSystem.builder()
            .posPid(headingPIDCoefficients)
            .build();

    @Override
    public void onInit() {
    }

    @Override
    public void onStartButtonPressed() {

    }

    @Override
    public void onUpdate() {
        ActiveOpMode.telemetry().addData("frontLeft", frontLeftMotor.getCurrentPosition());
        ActiveOpMode.telemetry().addData("frontRight", frontRightMotor.getCurrentPosition());
        ActiveOpMode.telemetry().addData("backLeft", backLeftMotor.getCurrentPosition());
        ActiveOpMode.telemetry().addData("backRight", backRightMotor.getCurrentPosition());
    }
}