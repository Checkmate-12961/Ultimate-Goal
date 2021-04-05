package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

@Config
public class LauncherConstants {
    public static int triggerActuationTime = 500;

    // everything
    public static double flywheelThreshold = 10; // In RPM

    // high goal
    public static double highGoalX = -1.5;
    public static double highGoalY = -22.5;
    public static double highGoalVelo = 5600; // 1; // roughly at a little over 12 volts
    public static double highGoalAngle = 0;

    // // TELE
    // power shot
    public static double powerShotAngle = 0; // angle the robot turns
    public static double powerShotVeloRight = 3500; // 0.72; //
    public static double powerShotVeloCenter = 3700; // 0.75; //
    public static double powerShotVeloLeft = 3800; // 0.72; //
    public static int shootCoolDown = 1000;

    // coordinates
    public static double powerShotX = -10; //X coordinate for the leftmost powershot
    public static double powerShotY = -20;   //Y coordinate for the leftmost powershot
    public static double pegDist = 7.7; // distance between each shot
    //public static double rotFix = 0;

    // // AUTO
    // power shot
    public static double autoPowerShotAngle = 0; // angle the robot turns
    public static double autoPowerShotVeloRight = 3700; // 0.72; //
    public static double autoPowerShotVeloCenter = 3700; // 0.75; //
    public static double autoPowerShotVeloLeft = 3700; // 0.72; //

    // coordinates
    public static double autoPowerShotX = -10;
    public static double autoPowerShotY = -20;
    public static double autoPegDist = 7.7; // distance between each shot
    //public static double autoRotFix = 0;

    public enum Position{LEFT, CENTER, RIGHT}

    public static Pose2d getPowerPose(Position pos){
        Pose2d poseToReturn;
        switch (pos){
            case RIGHT: poseToReturn = new Pose2d(
                    powerShotX,
                    powerShotY,
                    Math.toRadians(powerShotAngle)
            );
                break;
            case CENTER: poseToReturn = new Pose2d(powerShotX,
                    powerShotY + pegDist,
                    Math.toRadians(powerShotAngle /*+ rotFix*/)
            );
                break;
            case LEFT: poseToReturn = new Pose2d(
                    powerShotX,
                    powerShotY + pegDist * 2,
                    Math.toRadians(powerShotAngle /*+ rotFix * 2*/)
            );
                break;
            default: poseToReturn = null;
                break;
        }
        return poseToReturn;
    }

    public static Pose2d autoGetPowerPose(Position pos){
        Pose2d poseToReturn;
        switch (pos){
            case RIGHT: poseToReturn = new Pose2d(
                    autoPowerShotX,
                    autoPowerShotY,
                    Math.toRadians(autoPowerShotAngle)
                );
                break;
            case CENTER: poseToReturn = new Pose2d(autoPowerShotX,
                    autoPowerShotY + autoPegDist,
                    Math.toRadians(autoPowerShotAngle /*+ autoRotFix*/)
                );
                break;
            case LEFT: poseToReturn = new Pose2d(
                    autoPowerShotX,
                    autoPowerShotY + autoPegDist * 2,
                    Math.toRadians(autoPowerShotAngle /*+ autoRotFix * 2*/)
                );
                break;
            default: poseToReturn = null;
                break;
        }
        return poseToReturn;
    }
}
