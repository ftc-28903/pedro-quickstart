package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

@Configurable
@TeleOp(name="HoodServoCalibrate")
public class HoodServoCalibrate extends NextFTCOpMode {
    public HoodServoCalibrate() {
        addComponents(
                BindingsComponent.INSTANCE
        );
    }

    private final ServoEx hoodServo1 = new ServoEx("blocker");
    //private final MotorEx motor1 = new MotorEx("1");
    //private final MotorEx motor2 = new MotorEx("2");
    //private final MotorEx motor3 = new MotorEx("3");
    //private final MotorGroup shooter = new MotorGroup(motor2,motor3);

    public static double servoPos = 0.5;
    public static double usPulseLower = 1100;
    public static double usPulseUpper = 1800;

    @Override
    public void onInit() {
        hoodServo1.getServo().setDirection(Servo.Direction.FORWARD);
        PwmControl hoodServo1PWM = (PwmControl) hoodServo1.getServo();
        hoodServo1PWM.setPwmRange(new PwmControl.PwmRange(usPulseLower, usPulseUpper, 10000));
    }

    @Override
    public void onStartButtonPressed() {
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
        hoodServo1.setPosition(servoPos);
    }
}
