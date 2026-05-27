package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

@Configurable
public class Transfer implements Subsystem {
    public static final Transfer INSTANCE = new Transfer();
    public static double detectDist = 40;
    public static double maxMotorSpeed = 1;
    public static double maxOverrideSpeed = 1;
    public static double readDelay = 0;
    private Transfer() {}

    public ElapsedTime colorGetTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public double lastDistance = 0.0;
    public boolean override = false;
    public boolean opModeOverride = false;
    public boolean offOverride = false;
    public final MotorEx motor1 = new MotorEx("intake2").reversed();
    private final ServoEx blockerServo = new ServoEx("blocker");
    public RevColorSensorV3 colorSensorV3;

    public ElapsedTime overrideCycleTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public Command overrideOn = new InstantCommand(() -> {
        override = true;
        overrideCycleTimer.reset();
    });
    public Command spinUpReverse = new InstantCommand(() -> motor1.setPower(-1));

    public Command overrideOff = new InstantCommand(() -> override = false);
    public Command opModeOverrideOn = new InstantCommand(() -> opModeOverride = true);
    public Command opModeOverrideOff = new InstantCommand(() -> opModeOverride = false);

    public Command offOverrideOn = new InstantCommand(() -> offOverride = true);
    public Command offOverrideOff = new InstantCommand(() -> offOverride = false);


    @Override
    public void initialize() {
        colorSensorV3 = ActiveOpMode.hardwareMap().get(RevColorSensorV3.class, "color_sensor");

        blockerServo.getServo().setDirection(Servo.Direction.FORWARD);
        PwmControl blockerServoPWM = (PwmControl) blockerServo.getServo();
        // TODO: pwm range
        blockerServoPWM.setPwmRange(new PwmControl.PwmRange(500, 2500, 10000));
    }

    @Override
    public void periodic() {
        if (colorGetTimer.milliseconds() > readDelay) {
            lastDistance = colorSensorV3.getDistance(DistanceUnit.MM);
            colorGetTimer.reset();
        }

        ActiveOpMode.telemetry().addData("lastDistance", lastDistance);
        ActiveOpMode.telemetry().addData("transfer power", motor1.getPower());
        ActiveOpMode.telemetry().addData("is speed good", Shooter.INSTANCE.isSpeedGood());
        ActiveOpMode.telemetry().addData("intake2 amp", motor1.getMotor().getCurrent(CurrentUnit.MILLIAMPS));
        ActiveOpMode.telemetry().addData("blocker position", blockerServo.getPosition());

        if (offOverride) {
            blockerServo.setPosition(0);
            motor1.setPower(0);
            return;
        }

        if ((override || opModeOverride)) {
            blockerServo.setPosition(1);
            motor1.setPower(maxOverrideSpeed);

            return;
        }

        if (lastDistance > detectDist) {
            motor1.setPower(maxMotorSpeed);
            blockerServo.setPosition(0);
            return;
        }

        blockerServo.setPosition(0);

        motor1.setPower(0);
    }
}
