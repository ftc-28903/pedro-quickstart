package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class TrajectoryFactory {
    public static TrajectoryFactory INSTANCE = new TrajectoryFactory();

    // Define Poses from your original PathChain
    public Pose startPose = new Pose(15.500, 112.400, Math.toRadians(180));
    Pose farStartPose = new Pose(55.3, 8.5, Math.toRadians(180));
    Pose farParkPose = new Pose(41, 20);
    Pose shootPose1 = new Pose(50.000, 83.000);
    Pose intakeLine1Pose = new Pose(14.0000, 75.000);
    Pose intakeLine2Pose = new Pose(4.000, 60.000);
    Pose shootPose2 = new Pose(62.000, 76.000);
    Pose gateOpenIntakePose = new Pose(8.2, 60);
    Pose intakeLine3Pose = new Pose(4.0, 35.0);
    Pose farShootPose = new Pose(60, 20);

    Pose intakeLine3CurvePose = new Pose(62, 30);

    // PathChains (Will hold either Normal or Mirrored paths based on selection)
    public PathChain startFarIntakeLine3;
    public PathChain intakeLine3FarShoot;
    public PathChain farShootIntakeLine3;
    public PathChain farShootFarPark;
    public PathChain goalStartShoot;
    public PathChain shootIntakeLine1;
    public PathChain intakeLine1Shoot;
    public PathChain shootLine2Intake;
    public PathChain intakeLine2Shoot;
    public PathChain shoot2GateOpenIntake;
    public PathChain gateOpenIntakeShoot2;
    public PathChain startShoot2;
    public PathChain shoot2IntakeLine3;
    public PathChain intakeLine3Shoot2;

    public void buildTrajectories(Follower follower) {
        // Path 1: Curve to shootPose1
        goalStartShoot = follower.pathBuilder().addPath(
                        new BezierCurve(
                                startPose,
                                new Pose(41.419, 100.892),
                                shootPose1
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        // Path 2: Line to intakeLine1Pose
        shootIntakeLine1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                shootPose1,
                                new Pose(30,84),
                                intakeLine1Pose
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        // Path 3: Line back to shootPose2 with a heading turn
        intakeLine1Shoot = follower.pathBuilder().addPath(
                        new BezierLine(
                                intakeLine1Pose,
                                shootPose2
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(220))
                .build();

        // Path 4: Curve to intakeLine2Pose
        shootLine2Intake = follower.pathBuilder().addPath(
                        new BezierCurve(
                                shootPose2,
                                new Pose(46, 58),
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
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        // Path 6: Line to gateOpenIntakePose
        shoot2GateOpenIntake = follower.pathBuilder().addPath(
                        new BezierLine(
                                shootPose2,
                                gateOpenIntakePose
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(155))
                .build();

        // Path 7: Curve back to shootPose2
        gateOpenIntakeShoot2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                gateOpenIntakePose,
                                new Pose(20, 58),
                                shootPose2
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(155), Math.toRadians(180))
                .build();

        // Path 8: Direct line from start to shootPose2
        startShoot2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                startPose,
                                shootPose2
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        startFarIntakeLine3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                farStartPose,
                                intakeLine3CurvePose,
                                intakeLine3Pose
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        intakeLine3FarShoot = follower.pathBuilder().addPath(
                        new BezierLine(
                                intakeLine3Pose,
                                farShootPose
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        farShootIntakeLine3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                farShootPose,
                                intakeLine3CurvePose,
                                intakeLine3Pose
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        shoot2IntakeLine3 = follower.pathBuilder().addPath(
                new BezierCurve(
                        shootPose2,
                        intakeLine3CurvePose,
                        intakeLine3Pose
                )
        ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        intakeLine3Shoot2 = follower.pathBuilder().addPath(
                new BezierLine(
                        intakeLine3Pose,
                        shootPose2
                )
        ).setConstantHeadingInterpolation(180)
                .build();

        farShootFarPark = follower.pathBuilder().addPath(
                        new BezierLine(
                                farShootPose,
                                farParkPose
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    public void buildMirroredTrajectories(Follower follower) {
        // Mirrored Poses
        Pose startPoseMirrored = startPose.mirror();
        Pose farStartPoseMirrored = farStartPose.mirror();
        Pose farParkPoseMirrored = farParkPose.mirror();
        Pose shootPose1Mirrored = shootPose1.mirror();
        Pose intakeLine1PoseMirrored = intakeLine1Pose.mirror();
        Pose intakeLine2PoseMirrored = intakeLine2Pose.mirror();
        Pose shootPose2Mirrored = shootPose2.mirror();
        Pose gateOpenIntakePoseMirrored = gateOpenIntakePose.mirror();
        Pose intakeLine3PoseMirrored = intakeLine3Pose.mirror();
        Pose farShootPoseMirrored = farShootPose.mirror();

        // Mirrored Control Points
        Pose goalStartShootControlMirrored = new Pose(41.419, 100.892).mirror();
        Pose shootLine2IntakeControlMirrored = new Pose(46, 58).mirror();
        Pose intakeLine2ShootControlMirrored = new Pose(20.824, 47.696).mirror();
        Pose gateOpenIntakeShoot2ControlMirrored = new Pose(20, 58).mirror();
        Pose intakeLine3CurvePoseMirrored = intakeLine3CurvePose.mirror();

        // Mirrored Headings dynamically sourced from Pedro's mirror math
        double heading180Mirrored = startPoseMirrored.getHeading();
        double heading220Mirrored = new Pose(0, 0, Math.toRadians(220)).mirror().getHeading();
        double heading155Mirrored = new Pose(0, 0, Math.toRadians(155)).mirror().getHeading();

        // 1. goalStartShoot Mirrored
        goalStartShoot = follower.pathBuilder().addPath(
                        new BezierCurve(
                                startPoseMirrored,
                                goalStartShootControlMirrored,
                                shootPose1Mirrored
                        )
                ).setLinearHeadingInterpolation(heading180Mirrored, heading180Mirrored)
                .build();

        // 2. shootIntakeLine1 Mirrored
        shootIntakeLine1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                shootPose1Mirrored,
                                intakeLine1PoseMirrored
                        )
                ).setConstantHeadingInterpolation(heading180Mirrored)
                .build();

        // 3. intakeLine1Shoot Mirrored
        intakeLine1Shoot = follower.pathBuilder().addPath(
                        new BezierLine(
                                intakeLine1PoseMirrored,
                                shootPose2Mirrored
                        )
                ).setLinearHeadingInterpolation(heading180Mirrored, heading220Mirrored)
                .build();

        // 4. shootLine2Intake Mirrored
        shootLine2Intake = follower.pathBuilder().addPath(
                        new BezierCurve(
                                shootPose2Mirrored,
                                shootLine2IntakeControlMirrored,
                                intakeLine2PoseMirrored
                        )
                ).setLinearHeadingInterpolation(heading220Mirrored, heading180Mirrored)
                .build();

        // 5. intakeLine2Shoot Mirrored
        intakeLine2Shoot = follower.pathBuilder().addPath(
                        new BezierCurve(
                                intakeLine2PoseMirrored,
                                intakeLine2ShootControlMirrored,
                                shootPose2Mirrored
                        )
                ).setConstantHeadingInterpolation(heading180Mirrored)
                .build();

        // 6. shoot2GateOpenIntake Mirrored
        shoot2GateOpenIntake = follower.pathBuilder().addPath(
                        new BezierLine(
                                shootPose2Mirrored,
                                gateOpenIntakePoseMirrored
                        )
                ).setLinearHeadingInterpolation(heading180Mirrored, heading155Mirrored)
                .build();

        // 7. gateOpenIntakeShoot2 Mirrored
        gateOpenIntakeShoot2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                gateOpenIntakePoseMirrored,
                                gateOpenIntakeShoot2ControlMirrored,
                                shootPose2Mirrored
                        )
                ).setLinearHeadingInterpolation(heading155Mirrored, heading180Mirrored)
                .build();

        // 8. startShoot2 Mirrored
        startShoot2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                startPoseMirrored,
                                shootPose2Mirrored
                        )
                ).setConstantHeadingInterpolation(heading180Mirrored)
                .build();

        // 9. startFarIntakeLine3 Mirrored
        startFarIntakeLine3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                farStartPoseMirrored,
                                intakeLine3CurvePoseMirrored,
                                intakeLine3PoseMirrored
                        )
                ).setConstantHeadingInterpolation(heading180Mirrored)
                .build();

        // 10. intakeLine3FarShoot Mirrored
        intakeLine3FarShoot = follower.pathBuilder().addPath(
                        new BezierLine(
                                intakeLine3PoseMirrored,
                                farShootPoseMirrored
                        )
                ).setConstantHeadingInterpolation(heading180Mirrored)
                .build();

        // 11. farShootIntakeLine3 Mirrored
        farShootIntakeLine3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                farShootPoseMirrored,
                                intakeLine3CurvePoseMirrored,
                                intakeLine3PoseMirrored
                        )
                ).setConstantHeadingInterpolation(heading180Mirrored)
                .build();

        // 12. farShootFarPark Mirrored
        farShootFarPark = follower.pathBuilder().addPath(
                        new BezierLine(
                                farShootPoseMirrored,
                                farParkPoseMirrored
                        )
                ).setConstantHeadingInterpolation(heading180Mirrored)
                .build();
    }
}