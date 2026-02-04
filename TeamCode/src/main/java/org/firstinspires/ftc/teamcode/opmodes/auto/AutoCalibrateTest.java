package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.Transfer;
import org.firstinspires.ftc.teamcode.subsystem.Webcam;

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

@Autonomous(name = "AutoCalibrateTest")
public class AutoCalibrateTest extends NextFTCOpMode {
    public AutoCalibrateTest() {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new SubsystemComponent(Shooter.INSTANCE, Intake.INSTANCE, Transfer.INSTANCE, Webcam.INSTANCE)
        );
    }

    private final IMUEx imu = new IMUEx("imu", Direction.UP, Direction.FORWARD);

    private final ElapsedTime loopTimeTimer = new ElapsedTime();

    private final MotorEx frontLeftMotor = new MotorEx("front_left").brakeMode();
    private final MotorEx frontRightMotor = new MotorEx("front_right").reversed().brakeMode();
    private final MotorEx backLeftMotor = new MotorEx("back_left").brakeMode();
    private final MotorEx backRightMotor = new MotorEx("back_right").reversed().brakeMode();

    private TelemetryManager telemetryM;

    private double slowModeStep = 0.25;

    private boolean slowMode = false;
    private double slowModeMultiplier = 0.25;

    private double beginPos = backRightMotor.getCurrentPosition();

    private ElapsedTime timer1 = new ElapsedTime();
    private ElapsedTime timer2 = new ElapsedTime();

    private TestAutoFactory testAutoFactory;

    private int state = 1;

    public void runAllMotors(double fl, double fr, double bl, double br) {
        frontLeftMotor.setPower(fl);
        frontRightMotor.setPower(fr);
        backLeftMotor.setPower(bl);
        backRightMotor.setPower(br);
    }

    @Override
    public void onInit() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        Webcam.INSTANCE.init();

        testAutoFactory = new TestAutoFactory(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, imu);
    }

    @Override
    public void onStartButtonPressed() {
        imu.zero();
        testAutoFactory.getAutoGroup().schedule();
    }

    @Override
    public void onUpdate() {
        telemetryM.update(telemetry);
        testAutoFactory.loop();



        telemetryM.addData("backRightPos", backRightMotor.getCurrentPosition());
        loopTimeTimer.reset();
    }
}