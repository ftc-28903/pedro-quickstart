package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.ivy.Scheduler.schedule;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.Transfer;
import org.firstinspires.ftc.teamcode.subsystem.Turret;
import org.firstinspires.ftc.teamcode.subsystem.Webcam;
import org.firstinspires.ftc.teamcode.utils.AllianceColor;
import org.firstinspires.ftc.teamcode.utils.AutoMode;
import org.firstinspires.ftc.teamcode.utils.AutoStorage;
import org.firstinspires.ftc.teamcode.utils.CGHelpers;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "PedroFarAuto")
public class PedroFarAuto extends NextFTCOpMode {
    public PedroFarAuto() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE, Transfer.INSTANCE, Turret.INSTANCE, Webcam.INSTANCE),
                BulkReadComponent.INSTANCE
        );
    }

    public Follower follower;
    private TelemetryManager telemetryM;

    @Override
    public void onInit() {
        AutoStorage.autoMode = AutoMode.FAR;
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        Scheduler.reset();
        follower = Constants.createFollower(hardwareMap);
        if (AutoStorage.allianceColor == AllianceColor.BLUE) {
            TrajectoryFactory.INSTANCE.buildTrajectories(follower);
            follower.setStartingPose(TrajectoryFactory.INSTANCE.farStartPose);
        } else {
            TrajectoryFactory.INSTANCE.buildMirroredTrajectories(follower);
            follower.setStartingPose(TrajectoryFactory.INSTANCE.farStartPose.mirror());
        }
        schedule(CGHelpers.getInitGroup());

        AutoStorage.opModeStarted = false;
    }

    @Override
    public void onStartButtonPressed() {
        AutoStorage.opModeStarted = true;
        AutoStorage.follower = follower;
        schedule(CGHelpers.getStartGroup());
        schedule(AutoRoutines.INSTANCE.getFarGroup(follower));
    }

    @Override
    public void onStop() {
        AutoStorage.prevOpmodeWasAuto = true;
        AutoStorage.autoEndPose = follower.getPose();
    }

    @Override
    public void onUpdate() {
        follower.update();
        Scheduler.execute();

        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
        telemetryM.update(telemetry);
    }
}
