package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class TrajectoryFactory {
    public static TrajectoryFactory INSTANCE = new TrajectoryFactory();

    Pose startPoseGoal = new Pose(23.667, 125.834, Math.toRadians(140));
    Pose shootPoseGoal = new Pose(65,80,Math.toRadians(140));
    Pose intakePoseGoal1 = new Pose(14, 80, Math.toRadians(180));
    Pose intakePoseGoal2 = new Pose(18, 58, Math.toRadians(180));
    Pose gatePrepare = new Pose(27, 71, Math.toRadians(180));
    Pose gateOpen = new Pose(15, 69, Math.toRadians(180));
    Pose intakePoseGoal3 = new Pose(17, 34, Math.toRadians(180));
    Pose parkPoseGoal = new Pose(63, 64, Math.toRadians(140));

    public PathChain goalShoot;
    public PathChain goalIntake1;
    public PathChain goalGatePrepare;
    public PathChain goalGateOpen;
    public PathChain goalGateOpenShoot;
    public PathChain goalIntake2;
    public PathChain goalIntake2Shoot;
    public PathChain goalIntake3;
    public PathChain goalIntake3Shoot;
    public PathChain goalPark;

    // Mirrored versions of all paths
    public PathChain goalShootMirrored;
    public PathChain goalIntake1Mirrored;
    public PathChain goalGatePrepareMirrored;
    public PathChain goalGateOpenMirrored;
    public PathChain goalGateOpenShootMirrored;
    public PathChain goalIntake2Mirrored;
    public PathChain goalIntake2ShootMirrored;
    public PathChain goalIntake3Mirrored;
    public PathChain goalIntake3ShootMirrored;
    public PathChain goalParkMirrored;

    public void buildTrajectories(Follower follower) {
        buildNormalTrajectories(follower);
        buildMirroredTrajectories(follower);
    }

    public void buildNormalTrajectories(Follower follower) {
        goalShoot = follower.pathBuilder().addPath(
                        new BezierLine(
                                startPoseGoal,
                                shootPoseGoal
                        )
                ).setLinearHeadingInterpolation(startPoseGoal.getHeading(), shootPoseGoal.getHeading())
                .build();

        goalIntake1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                shootPoseGoal,
                                intakePoseGoal1
                        )
                ).setLinearHeadingInterpolation(shootPoseGoal.getHeading(), intakePoseGoal1.getHeading())
                .build();

        goalGatePrepare = follower.pathBuilder().addPath(
                        new BezierLine(
                                intakePoseGoal1,
                                gatePrepare
                        )
                ).setConstantHeadingInterpolation(gatePrepare.getHeading())
                .build();

        goalGateOpen = follower.pathBuilder().addPath(
                        new BezierLine(
                                gatePrepare,
                                gateOpen
                        )
                ).setConstantHeadingInterpolation(gateOpen.getHeading())
                .build();

        goalGateOpenShoot = follower.pathBuilder().addPath(
                        new BezierLine(
                                gateOpen,
                                shootPoseGoal
                        )
                ).setLinearHeadingInterpolation(gateOpen.getHeading(), shootPoseGoal.getHeading())
                .build();

        goalIntake2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                shootPoseGoal,
                                new Pose(68.234, 55.564),
                                intakePoseGoal2
                        )
                ).setLinearHeadingInterpolation(shootPoseGoal.getHeading(), intakePoseGoal2.getHeading())
                .build();

        goalIntake2Shoot = follower.pathBuilder().addPath(
                        new BezierLine(
                                intakePoseGoal2,
                                shootPoseGoal
                        )
                ).setLinearHeadingInterpolation(intakePoseGoal2.getHeading(), shootPoseGoal.getHeading())
                .build();

        goalIntake3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                shootPoseGoal,
                                new Pose(58.553, 20.039),
                                intakePoseGoal3
                        )
                ).setLinearHeadingInterpolation(shootPoseGoal.getHeading(), intakePoseGoal3.getHeading())
                .build();

        goalIntake3Shoot = follower.pathBuilder().addPath(
                        new BezierLine(
                                intakePoseGoal3,
                                shootPoseGoal
                        )
                ).setLinearHeadingInterpolation(intakePoseGoal3.getHeading(), shootPoseGoal.getHeading())
                .build();

        goalPark = follower.pathBuilder().addPath(
                        new BezierLine(
                                shootPoseGoal,
                                parkPoseGoal
                        )
                ).setConstantHeadingInterpolation(parkPoseGoal.getHeading())
                .build();
    }

    public void buildMirroredTrajectories(Follower follower) {
        // Create mirrored poses
        Pose startPoseGoalMirrored = startPoseGoal.mirror();
        Pose shootPoseGoalMirrored = shootPoseGoal.mirror();
        Pose intakePoseGoal1Mirrored = intakePoseGoal1.mirror();
        Pose intakePoseGoal2Mirrored = intakePoseGoal2.mirror();
        Pose gatePrepareMirrored = gatePrepare.mirror();
        Pose gateOpenMirrored = gateOpen.mirror();
        Pose intakePoseGoal3Mirrored = intakePoseGoal3.mirror();
        Pose parkPoseGoalMirrored = parkPoseGoal.mirror();

        // Mirror the control points too
        Pose controlPoint2Mirrored = new Pose(68.234, 55.564, 0).mirror();
        Pose controlPoint3Mirrored = new Pose(58.553, 20.039, 0).mirror();

        goalShootMirrored = follower.pathBuilder().addPath(
                        new BezierLine(
                                startPoseGoalMirrored,
                                shootPoseGoalMirrored
                        )
                ).setLinearHeadingInterpolation(startPoseGoalMirrored.getHeading(), shootPoseGoalMirrored.getHeading())
                .build();

        goalIntake1Mirrored = follower.pathBuilder().addPath(
                        new BezierLine(
                                shootPoseGoalMirrored,
                                intakePoseGoal1Mirrored
                        )
                ).setLinearHeadingInterpolation(shootPoseGoalMirrored.getHeading(), intakePoseGoal1Mirrored.getHeading())
                .build();

        goalGatePrepareMirrored = follower.pathBuilder().addPath(
                        new BezierLine(
                                intakePoseGoal1Mirrored,
                                gatePrepareMirrored
                        )
                ).setConstantHeadingInterpolation(gatePrepareMirrored.getHeading())
                .build();

        goalGateOpenMirrored = follower.pathBuilder().addPath(
                        new BezierLine(
                                gatePrepareMirrored,
                                gateOpenMirrored
                        )
                ).setConstantHeadingInterpolation(gateOpenMirrored.getHeading())
                .build();

        goalGateOpenShootMirrored = follower.pathBuilder().addPath(
                        new BezierLine(
                                gateOpenMirrored,
                                shootPoseGoalMirrored
                        )
                ).setLinearHeadingInterpolation(gateOpenMirrored.getHeading(), shootPoseGoalMirrored.getHeading())
                .build();

        goalIntake2Mirrored = follower.pathBuilder().addPath(
                        new BezierCurve(
                                shootPoseGoalMirrored,
                                controlPoint2Mirrored,
                                intakePoseGoal2Mirrored
                        )
                ).setLinearHeadingInterpolation(shootPoseGoalMirrored.getHeading(), intakePoseGoal2Mirrored.getHeading())
                .build();

        goalIntake2ShootMirrored = follower.pathBuilder().addPath(
                        new BezierLine(
                                intakePoseGoal2Mirrored,
                                shootPoseGoalMirrored
                        )
                ).setLinearHeadingInterpolation(intakePoseGoal2Mirrored.getHeading(), shootPoseGoalMirrored.getHeading())
                .build();

        goalIntake3Mirrored = follower.pathBuilder().addPath(
                        new BezierCurve(
                                shootPoseGoalMirrored,
                                controlPoint3Mirrored,
                                intakePoseGoal3Mirrored
                        )
                ).setLinearHeadingInterpolation(shootPoseGoalMirrored.getHeading(), intakePoseGoal3Mirrored.getHeading())
                .build();

        goalIntake3ShootMirrored = follower.pathBuilder().addPath(
                        new BezierLine(
                                intakePoseGoal3Mirrored,
                                shootPoseGoalMirrored
                        )
                ).setLinearHeadingInterpolation(intakePoseGoal3Mirrored.getHeading(), shootPoseGoalMirrored.getHeading())
                .build();

        goalParkMirrored = follower.pathBuilder().addPath(
                        new BezierLine(
                                shootPoseGoalMirrored,
                                parkPoseGoalMirrored
                        )
                ).setConstantHeadingInterpolation(parkPoseGoalMirrored.getHeading())
                .build();
    }
}