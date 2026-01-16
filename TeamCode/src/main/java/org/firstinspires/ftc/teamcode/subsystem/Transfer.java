package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
public class Transfer implements Subsystem {
    public static final Transfer INSTANCE = new Transfer();
    public static double detectDist = 120;
    public static double maxMotorSpeed = 0.75;
    public static double maxOverrideSpeed = 1;
    public static double readDelay = 150;
    private Transfer() {}

    public ElapsedTime colorGetTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public double lastDistance = 9999.9;
    public boolean override = false;
    public boolean opModeOverride = false;
    public boolean offOverride = false;
    public final MotorEx motor1 = new MotorEx("transfer1");
    public RevColorSensorV3 colorSensorV3;

    public Command overrideOn = new InstantCommand(() -> override = true);
    public Command spinUpReverse = new InstantCommand(() -> motor1.setPower(-1));

    public Command overrideOff = new InstantCommand(() -> override = false);
    public Command opModeOverrideOn = new InstantCommand(() -> opModeOverride = true);
    public Command opModeOverrideOff = new InstantCommand(() -> opModeOverride = false);

    public Command offOverrideOn = new InstantCommand(() -> offOverride = true);
    public Command offOverrideOff = new InstantCommand(() -> offOverride = false);

    @Override
    public void initialize() {
        colorSensorV3 = ActiveOpMode.hardwareMap().get(RevColorSensorV3.class, "color_sensor");
    }

    @Override
    public void periodic() {
        if (colorGetTimer.milliseconds() > readDelay) {
            lastDistance = colorSensorV3.getDistance(DistanceUnit.MM);
            colorGetTimer.reset();
        }

        ActiveOpMode.telemetry().addData("lastDistance", lastDistance);

        if (offOverride) {
            motor1.setPower(0);
            return;
        }

        if (override || opModeOverride) {
            motor1.setPower(maxOverrideSpeed);
            return;
        }

        if (lastDistance > detectDist) {
            motor1.setPower(maxMotorSpeed);
        } else {
            motor1.setPower(0);
        }
    }
}
