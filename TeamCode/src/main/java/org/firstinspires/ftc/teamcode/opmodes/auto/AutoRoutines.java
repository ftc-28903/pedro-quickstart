package org.firstinspires.ftc.teamcode.opmodes.auto;

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.Transfer;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.CommandGroup;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.extensions.pedro.FollowPath;

public class AutoRoutines {
    public static AutoRoutines INSTANCE = new AutoRoutines();
    public int shootDelay = 2000;
    public int gateOpenDelay = 3000;
    public CommandGroup getTwelveattemptgroup() {
        return new SequentialGroup(
                Shooter.INSTANCE.spinUp,

                // preload
                new FollowPath(TrajectoryFactory.INSTANCE.goalShoot, true),
                new ParallelGroup(
                        Intake.INSTANCE.spinUp,
                        Transfer.INSTANCE.opModeOverrideOn
                ),
                new Delay(shootDelay),
                Transfer.INSTANCE.opModeOverrideOff,

                // intake 1 + gate open
                new FollowPath(TrajectoryFactory.INSTANCE.goalIntake1, true),
                Intake.INSTANCE.spinDown,
                new FollowPath(TrajectoryFactory.INSTANCE.goalGatePrepare, true),
                new FollowPath(TrajectoryFactory.INSTANCE.goalGateOpen, true),
                new Delay(gateOpenDelay),
                new FollowPath(TrajectoryFactory.INSTANCE.goalGateOpenShoot, true),
                Intake.INSTANCE.spinUp,
                Transfer.INSTANCE.opModeOverrideOn,
                new Delay(shootDelay),
                Transfer.INSTANCE.opModeOverrideOff,

                // intake 2
                new FollowPath(TrajectoryFactory.INSTANCE.goalIntake2, true),
                new FollowPath(TrajectoryFactory.INSTANCE.goalIntake2Shoot, true),
                Transfer.INSTANCE.opModeOverrideOn,
                new Delay(shootDelay),
                Transfer.INSTANCE.opModeOverrideOff,

                // intake 3
                new FollowPath(TrajectoryFactory.INSTANCE.goalIntake3, true),
                new FollowPath(TrajectoryFactory.INSTANCE.goalIntake3Shoot, true),
                Transfer.INSTANCE.opModeOverrideOn,
                new Delay(shootDelay),
                Transfer.INSTANCE.opModeOverrideOff,

                new FollowPath(TrajectoryFactory.INSTANCE.goalPark, true)
        );
    }
}
