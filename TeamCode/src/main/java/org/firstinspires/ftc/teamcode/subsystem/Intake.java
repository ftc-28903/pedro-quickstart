package org.firstinspires.ftc.teamcode.subsystem;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;

import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class Intake implements Subsystem {
    public static final Intake INSTANCE = new Intake();
    private Intake() {}

    public final MotorEx motor1 = new MotorEx("intake1");

    public Command spinUp = Commands.instant(() -> motor1.setPower(1));
    public Command spinUpReverse = Commands.instant(() -> motor1.setPower(-1));

    public Command spinDown = Commands.instant(() -> motor1.setPower(0));
}
