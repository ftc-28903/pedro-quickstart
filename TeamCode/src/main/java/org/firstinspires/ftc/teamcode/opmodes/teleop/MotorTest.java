package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name="MotorTest")
public class MotorTest extends NextFTCOpMode {
    public MotorTest() {
        addComponents(
                BindingsComponent.INSTANCE
        );
    }

    private final MotorEx intake1 = new MotorEx("intake1");
    private final MotorEx intake2 = new MotorEx("intake2").reversed();
    //private final MotorEx motor1 = new MotorEx("1");
    //private final MotorEx motor2 = new MotorEx("2");
    //private final MotorEx motor3 = new MotorEx("3");
    //private final MotorGroup shooter = new MotorGroup(motor2,motor3);

    @Override
    public void onInit() {

    }

    @Override
    public void onStartButtonPressed() {
        intake1.setPower(1.0);
        intake2.setPower(0.6);
        //motor2.setPower(1);
        /*Gamepads.gamepad1().a()
                .whenBecomesTrue(() -> motor0.setPower(1))
                .whenBecomesFalse(() -> motor0.setPower(0));

        Gamepads.gamepad1().circle()
                .whenBecomesTrue(() -> motor1.setPower(1))
                .whenBecomesFalse(() -> motor1.setPower(0));

        Gamepads.gamepad1().triangle()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() -> shooter.setPower(0.7))
                .whenBecomesFalse(() -> shooter.setPower(0));*/
    }

    @Override
    public void onUpdate() {
        //ActiveOpMode.telemetry().addData("shooter1 rotations", shooter1.getCurrentPosition());
        ActiveOpMode.telemetry().update();
    }
}
