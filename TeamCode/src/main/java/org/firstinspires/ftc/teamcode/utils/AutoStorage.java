package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

public class AutoStorage {
    public static AllianceColor allianceColor = AllianceColor.BLUE;
    public static boolean isFirstRun = true;
    public static AutoMode autoMode = AutoMode.CLOSE;
    public static Follower follower;
    public static boolean prevOpmodeWasAuto = false;
    public static Pose autoEndPose;
    public static boolean opModeStarted = false;
}
