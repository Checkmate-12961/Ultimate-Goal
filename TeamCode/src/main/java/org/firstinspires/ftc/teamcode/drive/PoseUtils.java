package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

@Config
public class PoseUtils {
    // Simple class to transfer position data between opmodes
    public static volatile Pose2d currentPose = new Pose2d(0,0,0);

    public static double startX = -62;
    public static double startY = -19.5;
    public static double startHeading = -1.9;

    public static Pose2d getStartPose () {
        return new Pose2d(startX, startY, Math.toRadians(startHeading));
    }
}
