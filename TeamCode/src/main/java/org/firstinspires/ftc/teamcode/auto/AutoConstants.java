package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

@Config
public class AutoConstants {
    //These are the coordinates for the wobble goal placements.
    public static double dropAX = 6;
    public static double dropAY = -42;
    public static double dropAH = 90;

    public static double dropBX = 12;
    public static double dropBY = -24;
    public static double dropBH = 120;

    public static double dropCX = 40;
    public static double dropCY = -42;
    public static double dropCH = 120;

    public enum Box {A,B,C}

    public static Pose2d getBoxPose(Box box) {
        Pose2d poseToReturn;
        switch (box){
            case A: poseToReturn = new Pose2d(
                    dropAX,
                    dropAY,
                    Math.toRadians(dropAH)
            );
                break;
            case B: poseToReturn = new Pose2d(
                    dropBX,
                    dropBY,
                    Math.toRadians(dropBH)
            );
                break;
            case C: poseToReturn = new Pose2d(
                    dropCX,
                    dropCY,
                    Math.toRadians(dropCH)
            );
                break;
            default: poseToReturn = null;
                break;
        }
        return poseToReturn;
    }
}
