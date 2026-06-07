package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.Transfer;
import org.firstinspires.ftc.teamcode.subsystem.Turret;
import org.firstinspires.ftc.teamcode.subsystem.Webcam;
import org.firstinspires.ftc.teamcode.utils.AllianceColor;
import org.firstinspires.ftc.teamcode.utils.AutoStorage;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;

@Autonomous(name = "Far zone auto")
public class AutoCalibrateTest extends NextFTCOpMode {
    public AutoCalibrateTest() {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new SubsystemComponent(Shooter.INSTANCE, Intake.INSTANCE, Transfer.INSTANCE, Webcam.INSTANCE, Turret.INSTANCE)
        );
    }

    private final IMUEx imu = new IMUEx("imu", Direction.UP, Direction.FORWARD);

    private final ElapsedTime loopTimeTimer = new ElapsedTime();

    private final MotorEx frontLeftMotor = new MotorEx("front_left").floatMode();
    private final MotorEx frontRightMotor = new MotorEx("front_right").reversed().floatMode();
    private final MotorEx backLeftMotor = new MotorEx("back_left").floatMode();
    private final MotorEx backRightMotor = new MotorEx("back_right").reversed().floatMode();

    private TelemetryManager telemetryM;
    private TestAutoFactory testAutoFactory;

    @Override
    public void onInit() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        Webcam.INSTANCE.init();

        imu.zero();
        testAutoFactory = new TestAutoFactory(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, imu);

        Intake.INSTANCE.spinDown.schedule();
        Shooter.INSTANCE.spinDown.schedule();
        Transfer.INSTANCE.overrideOff.schedule();

        Transfer.INSTANCE.offOverrideOn.schedule();

        if (AutoStorage.allianceColor == AllianceColor.RED) {
            testAutoFactory.dirMultiplier = -1;
        } else if (AutoStorage.allianceColor == AllianceColor.BLUE) {
            testAutoFactory.dirMultiplier = 1;
        }
        Turret.INSTANCE.disableTurret.schedule();

        telemetry.addData("alliance", AutoStorage.allianceColor);
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        Transfer.INSTANCE.offOverrideOff.schedule();
        imu.zero();
        testAutoFactory.getFarzoneGroup().schedule();
    }

    @Override
    public void onUpdate() {
        telemetryM.update(telemetry);
        testAutoFactory.loop();

        telemetryM.addData("backRightPos", backRightMotor.getCurrentPosition());
        telemetryM.addData("imuDeg", imu.get().inDeg);
        loopTimeTimer.reset();
    }
}