package org.firstinspires.ftc.teamcode.utils;

import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.Transfer;
import org.firstinspires.ftc.teamcode.subsystem.Turret;

public class CGHelpers {
    public static Command getInitGroup() {
        return sequential(
                Intake.INSTANCE.spinDown,
                Shooter.INSTANCE.spinDown,
                Transfer.INSTANCE.overrideOff,
                Turret.INSTANCE.disableTurret,
                Transfer.INSTANCE.offOverrideOn
        );
    }

    public static Command getStartGroup() {
        return sequential(
                Shooter.INSTANCE.spinUp,
                Turret.INSTANCE.enableTurret,
                Transfer.INSTANCE.offOverrideOff
        );
    }
}
