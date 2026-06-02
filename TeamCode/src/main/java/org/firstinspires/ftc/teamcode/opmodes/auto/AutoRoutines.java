package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;

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
    public Command getTwelveattemptgroup(Follower follower) {
        return sequential(
                //Shooter.INSTANCE.spinUp,

                // preload
                follow(follower, TrajectoryFactory.INSTANCE.goalShoot),
                //new Delay(shootDelay),
                //Transfer.INSTANCE.opModeOverrideOff,

                // intake 1 + gate open
                follow(follower, TrajectoryFactory.INSTANCE.goalIntake1),
                //Intake.INSTANCE.spinDown,
                follow(follower, TrajectoryFactory.INSTANCE.goalGatePrepare),
                follow(follower, TrajectoryFactory.INSTANCE.goalGateOpen),
                //new Delay(gateOpenDelay),
                follow(follower, TrajectoryFactory.INSTANCE.goalGateOpenShoot),
                //Intake.INSTANCE.spinUp,
                //Transfer.INSTANCE.opModeOverrideOn,
                //new Delay(shootDelay),
                //Transfer.INSTANCE.opModeOverrideOff,

                // intake 2
                follow(follower, TrajectoryFactory.INSTANCE.goalIntake2),
                follow(follower, TrajectoryFactory.INSTANCE.goalIntake2Shoot),

                // intake 3
                follow(follower, TrajectoryFactory.INSTANCE.goalIntake3),
                follow(follower, TrajectoryFactory.INSTANCE.goalIntake3Shoot),
                //Transfer.INSTANCE.opModeOverrideOn,
                //new Delay(shootDelay),
                //Transfer.INSTANCE.opModeOverrideOff,

                follow(follower, TrajectoryFactory.INSTANCE.goalPark)
        );
    }
}