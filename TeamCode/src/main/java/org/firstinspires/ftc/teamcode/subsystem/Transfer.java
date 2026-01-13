package org.firstinspires.ftc.teamcode.subsystem;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class Transfer implements Subsystem {
    public static final Transfer INSTANCE = new Transfer();
    private Transfer() {}

    public final MotorEx motor1 = new MotorEx("transfer1").reversed();

    public Command spinUp = new InstantCommand(() -> motor1.setPower(1));
    public Command spinUpReverse = new InstantCommand(() -> motor1.setPower(-1));

    public Command spinDown = new InstantCommand(() -> motor1.setPower(0));
}
