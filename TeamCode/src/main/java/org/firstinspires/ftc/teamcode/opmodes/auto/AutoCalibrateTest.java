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

    private final MotorEx frontLeftMotor = new MotorEx("front_left");
    private final MotorEx frontRightMotor = new MotorEx("front_right").reversed();
    private final MotorEx backLeftMotor = new MotorEx("back_left");
    private final MotorEx backRightMotor = new MotorEx("back_right").reversed();

    private TelemetryManager telemetryM;

    private double slowModeStep = 0.25;

    private boolean slowMode = false;
    private double slowModeMultiplier = 0.25;

    private double beginPos = backRightMotor.getCurrentPosition();

    private ElapsedTime timer1 = new ElapsedTime();
    private ElapsedTime timer2 = new ElapsedTime();

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
    }

    @Override
    public void onStartButtonPressed() {
        beginPos = backRightMotor.getCurrentPosition();
        timer1.reset();
        timer2.reset();
        Shooter.INSTANCE.spinUp.schedule();
    }

    @Override
    public void onUpdate() {
        telemetryM.update(telemetry);
        double backRightPosDiff = Math.abs(beginPos-backRightMotor.getCurrentPosition());

        switch (state) {
            case 1:
                if (timer1.milliseconds() > 3000) {
                    runAllMotors(-0.5,-0.5,-0.5,-0.5);
                    Intake.INSTANCE.spinUp.schedule();
                    state = 2;
                }
            case 2:
                if (backRightPosDiff >= 1685) {
                    runAllMotors(0,0,0,0);
                    state = 3;
                    Intake.INSTANCE.spinUp.schedule();
                    Transfer.INSTANCE.opModeOverrideOn.schedule();
                    timer1.reset();
                }
            case 3:
                if (timer1.milliseconds() > 4000) {
                    Intake.INSTANCE.spinDown.schedule();
                    Transfer.INSTANCE.opModeOverrideOff.schedule();
                    Shooter.INSTANCE.spinDown.schedule();
                }
        }

        telemetryM.addData("slowMode toggle", slowMode);
        telemetryM.addData("slowMode multiplier", slowModeMultiplier);
        telemetryM.addData("loop time", loopTimeTimer.milliseconds());
        telemetryM.addData("backRightPos", beginPos-backRightMotor.getCurrentPosition());
        loopTimeTimer.reset();
    }
}