package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.commands.Commands.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.Transfer;

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
                Intake.INSTANCE.spinDown,
                Transfer.INSTANCE.overrideOff
        );
    }

    public static Command getFarShootCommand() {
        return sequential(
                getShootStartGroup(),
                waitMs(3500),
                getShootStopGroup()
        );
    }

    public static Command getShootCommand() {
        return sequential(
                getShootStartGroup(),
                waitMs(1200),
                getShootStopGroup()
        );
    }

    public Command getTestRoutine(Follower follower) {
        return sequential(
                Shooter.INSTANCE.spinDown,
                follow(follower, TrajectoryFactory.INSTANCE.startShoot2),
                Intake.INSTANCE.spinUp,
                follow(follower, TrajectoryFactory.INSTANCE.shoot2GateOpenIntake),
                waitMs(2000),
                follow(follower, TrajectoryFactory.INSTANCE.gateOpenIntakeShoot2)
        );
    }

    public Command getFarGroup(Follower follower) {
        return sequential(
                waitMs(500),
                Shooter.INSTANCE.waitForSpeed,
                waitMs(1000),
                getFarShootCommand(),
                follow(follower, TrajectoryFactory.INSTANCE.startFarIntakeLine3),
                follow(follower, TrajectoryFactory.INSTANCE.intakeLine3FarShoot),
                getFarShootCommand(),
                follow(follower, TrajectoryFactory.INSTANCE.farShootIntakeLine3),
                follow(follower, TrajectoryFactory.INSTANCE.intakeLine3FarShoot),
                getFarShootCommand(),
                follow(follower, TrajectoryFactory.INSTANCE.farShootIntakeLine3),
                follow(follower, TrajectoryFactory.INSTANCE.intakeLine3FarShoot),
                getFarShootCommand(),
                follow(follower, TrajectoryFactory.INSTANCE.farShootFarPark)
        );
    }

    public Command getTwelveattemptgroup(Follower follower) {
        return sequential(
                waitMs(500),
                // Preload: Curve to shooting position
                Shooter.INSTANCE.spinUp,
                follow(follower, TrajectoryFactory.INSTANCE.goalStartShoot),
                waitUntil(Shooter.INSTANCE::isSpeedGood),
                waitMs(1000),
                getShootCommand(),
                //Transfer.INSTANCE.overrideOff,
                // new Delay(shootDelay),

                // Intake 1: Line to first intake line
                Intake.INSTANCE.spinUp,
                follow(follower, TrajectoryFactory.INSTANCE.shootIntakeLine1),
                //Intake.INSTANCE.spinDown,

                // Shoot 1: Line back to shoot position with heading turn
                follow(follower, TrajectoryFactory.INSTANCE.intakeLine1Shoot),
                getShootCommand(),

                // Intake 2: Curve down to second intake line
                Intake.INSTANCE.spinUp,
                race(
                        follow(follower, TrajectoryFactory.INSTANCE.shootLine2Intake),
                        waitMs(3000)
                ),
                //Intake.INSTANCE.spinDown,

                // Shoot 2: Curve back to shooting position
                follow(follower, TrajectoryFactory.INSTANCE.intakeLine2Shoot),
                getShootCommand(),
                Intake.INSTANCE.spinUp,

                follow(follower, TrajectoryFactory.INSTANCE.shoot2GateOpenIntake),
                waitMs(2500),
                follow(follower, TrajectoryFactory.INSTANCE.gateOpenIntakeShoot2),
                waitMs(800),
                getShootCommand(),
                Intake.INSTANCE.spinUp,
                follow(follower, TrajectoryFactory.INSTANCE.shoot2GateOpenIntake),
                waitMs(1500),
                follow(follower, TrajectoryFactory.INSTANCE.gateOpenIntakeShoot2),
                waitMs(800),
                getShootCommand(),
                Intake.INSTANCE.spinUp,
                follow(follower, TrajectoryFactory.INSTANCE.shoot2GateOpenIntake),
                waitMs(1500),
                follow(follower, TrajectoryFactory.INSTANCE.gateOpenIntakeShoot2),
                waitMs(500),
                getShootCommand(),
                Intake.INSTANCE.spinUp
                // Transfer.INSTANCE.overrideOn,
                // new Delay(shootDelay),
        );
    }
}