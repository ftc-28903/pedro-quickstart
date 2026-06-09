package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.commands.Commands.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.Transfer;

import dev.nextftc.core.commands.delays.WaitUntil;

public class AutoRoutines {
    public static AutoRoutines INSTANCE = new AutoRoutines();
    public int shootDelay = 2000;
    public int gateOpenDelay = 3000;

    public static Command getShootStartGroup() {
        return parallel(
                Intake.INSTANCE.spinUp,
                Transfer.INSTANCE.overrideOn
        );
    }

    public static Command getShootStopGroup() {
        return parallel(
                //Intake.INSTANCE.spinDown,
                Transfer.INSTANCE.overrideOff
        );
    }

    public static Command getShootCommand() {
        return sequential(
                getShootStartGroup(),
                waitMs(800),
                getShootStopGroup()
        );
    }

    public Command getTwelveattemptgroup(Follower follower) {
        return sequential(
                // Preload: Curve to shooting position
                follow(follower, TrajectoryFactory.INSTANCE.goalStartShoot),
                waitUntil(Shooter.INSTANCE::isSpeedGood),
                waitMs(1000),
                getShootCommand(),
                //Transfer.INSTANCE.overrideOff,
                // new Delay(shootDelay),

                // Intake 1: Line to first intake line
                //Intake.INSTANCE.spinUp,
                follow(follower, TrajectoryFactory.INSTANCE.shootIntakeLine1),
                //Intake.INSTANCE.spinDown,

                // Shoot 1: Line back to shoot position with heading turn
                follow(follower, TrajectoryFactory.INSTANCE.intakeLine1Shoot),
                getShootCommand(),

                // Intake 2: Curve down to second intake line
                //Intake.INSTANCE.spinUp,
                follow(follower, TrajectoryFactory.INSTANCE.shootLine2Intake),
                //Intake.INSTANCE.spinDown,

                // Shoot 2: Curve back to shooting position
                follow(follower, TrajectoryFactory.INSTANCE.intakeLine2Shoot),
                getShootCommand(),

                follow(follower, TrajectoryFactory.INSTANCE.shoot2GateOpen),
                waitMs(300),

                follow(follower, TrajectoryFactory.INSTANCE.gateOpenGateIntake1),
                follow(follower, TrajectoryFactory.INSTANCE.gateIntake2Shoot2),
                getShootCommand(),

                follow(follower, TrajectoryFactory.INSTANCE.shoot2GateOpen),
                waitMs(300),

                follow(follower, TrajectoryFactory.INSTANCE.gateOpenGateIntake1),
                follow(follower, TrajectoryFactory.INSTANCE.gateIntake2Shoot2),
                getShootCommand(),

                follow(follower, TrajectoryFactory.INSTANCE.shoot2GateOpen),
                waitMs(300),

                follow(follower, TrajectoryFactory.INSTANCE.gateOpenGateIntake1),
                follow(follower, TrajectoryFactory.INSTANCE.gateIntake2Shoot2),
                getShootCommand()
                // Transfer.INSTANCE.overrideOn,
                // new Delay(shootDelay),
        );
    }
}