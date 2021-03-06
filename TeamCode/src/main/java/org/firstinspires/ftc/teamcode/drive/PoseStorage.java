package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class PoseStorage {
    // Simple class to transfer position data between opmodes
    public volatile Pose2d currentPose = new Pose2d(-58, -19.5 , Math.toRadians(0) );
    public static final Pose2d globalStartPose = new Pose2d(-62, -19.5, 0);
}
