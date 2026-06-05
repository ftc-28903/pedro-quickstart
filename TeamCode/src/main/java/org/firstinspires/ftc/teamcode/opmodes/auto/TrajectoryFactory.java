package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class TrajectoryFactory {
    public static TrajectoryFactory INSTANCE = new TrajectoryFactory();

    // Define Poses from your original PathChain
    public Pose startPose = new Pose(15.000, 109.000, Math.toRadians(180));
    Pose shootPose1 = new Pose(50.000, 80.000);
    Pose intakeLine1Pose = new Pose(15.5000, 80.000);
    Pose intakeLine2Pose = new Pose(2.000, 57.000);
    Pose shootPose2 = new Pose(61.000, 76.000);
    Pose gateOpenPose = new Pose(14,61);

    // PathChains (Normal)
    public PathChain goalStartShoot;
    public PathChain shootIntakeLine1;
    public PathChain intakeLine1Shoot;
    public PathChain shootLine2Intake;
    public PathChain intakeLine2Shoot;
    public PathChain shoot2GateOpen;

    // PathChains (Mirrored)
    public PathChain goalStartShootMirrored;
    public PathChain goalIntakeLine1Mirrored;
    public PathChain intakeLine1ShootMirrored;
    public PathChain shootLine2IntakeMirrored;
    public PathChain intakeLine2ShootMirrored;

    public void buildTrajectories(Follower follower) {
        buildNormalTrajectories(follower);
        buildMirroredTrajectories(follower);
    }

    public void buildNormalTrajectories(Follower follower) {
        // Path 1: Curve to shootPose1
        goalStartShoot = follower.pathBuilder().addPath(
                        new BezierCurve(
                                startPose,
                                new Pose(41.419, 100.892),
                                shootPose1
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        // Path 2: Line to intakeLine1Pose
        shootIntakeLine1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                shootPose1,
                                intakeLine1Pose
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        // Path 3: Line back to shootPose1 with a heading turn
        intakeLine1Shoot = follower.pathBuilder().addPath(
                        new BezierLine(
                                intakeLine1Pose,
                                shootPose1
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(220))
                .build();

        // Path 4: Curve to intakeLine2Pose
        shootLine2Intake = follower.pathBuilder().addPath(
                        new BezierCurve(
                                shootPose1,
                                new Pose(46, 50),
                                intakeLine2Pose
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(220), Math.toRadians(180))
                .build();

        // Path 5: Curve to shootPose2
        intakeLine2Shoot = follower.pathBuilder().addPath(
                        new BezierCurve(
                                intakeLine2Pose,
                                new Pose(20.824, 47.696),
                                shootPose2
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(210))
                .build();

        shoot2GateOpen = follower.pathBuilder().addPath(
                    new BezierLine(
                            shootPose2,
                            gateOpenPose
                    )
                ).setLinearHeadingInterpolation(Math.toRadians(210),Math.toRadians(180))
                .build();
    }

    public void buildMirroredTrajectories(Follower follower) {
        // Mirrored Poses
        Pose startPoseMirrored = new Pose(15.000, 109.000, Math.toRadians(180)).mirror();
        Pose shootPose1Mirrored = shootPose1.mirror();
        Pose intakeLine1PoseMirrored = intakeLine1Pose.mirror();
        Pose intakeLine2PoseMirrored = intakeLine2Pose.mirror();
        Pose shootPose2Mirrored = shootPose2.mirror();

        // Mirrored Control Points (Named after the paths they belong to)
        Pose goalStartShootControlMirrored = new Pose(41.419, 100.892, 0).mirror();
        Pose shootLine2IntakeControlMirrored = new Pose(60.595, 55.092, 0).mirror();
        Pose intakeLine2ShootControlMirrored = new Pose(20.824, 47.696, 0).mirror();

        // goalStartShoot Mirrored
        goalStartShootMirrored = follower.pathBuilder().addPath(
                        new BezierCurve(
                                startPoseMirrored,
                                goalStartShootControlMirrored,
                                shootPose1Mirrored
                        )
                ).setConstantHeadingInterpolation(startPoseMirrored.getHeading())
                .build();

        // goalIntakeLine1 Mirrored
        goalIntakeLine1Mirrored = follower.pathBuilder().addPath(
                        new BezierLine(
                                shootPose1Mirrored,
                                intakeLine1PoseMirrored
                        )
                ).setConstantHeadingInterpolation(startPoseMirrored.getHeading())
                .build();

        // intakeLine1Shoot Mirrored
        intakeLine1ShootMirrored = follower.pathBuilder().addPath(
                        new BezierLine(
                                intakeLine1PoseMirrored,
                                shootPose1Mirrored
                        )
                ).setLinearHeadingInterpolation(startPoseMirrored.getHeading(), new Pose(0, 0, Math.toRadians(220)).mirror().getHeading())
                .build();

        // shootLine2Intake Mirrored
        shootLine2IntakeMirrored = follower.pathBuilder().addPath(
                        new BezierCurve(
                                shootPose1Mirrored,
                                shootLine2IntakeControlMirrored,
                                intakeLine2PoseMirrored
                        )
                ).setLinearHeadingInterpolation(new Pose(0, 0, Math.toRadians(220)).mirror().getHeading(), startPoseMirrored.getHeading())
                .build();

        // intakeLine2Shoot Mirrored
        intakeLine2ShootMirrored = follower.pathBuilder().addPath(
                        new BezierCurve(
                                intakeLine2PoseMirrored,
                                intakeLine2ShootControlMirrored,
                                shootPose2Mirrored
                        )
                ).setConstantHeadingInterpolation(new Pose(0, 0, Math.toRadians(210)).mirror().getHeading())
                .build();
    }
}