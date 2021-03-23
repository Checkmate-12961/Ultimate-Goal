package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

@Config
public class LauncherConstants {
    public static int triggerActuationTime = 500;

    // everything
    public static double flywheelThreshold = 25; // In RPM

    // high goal
    public static double highGoalX = -1.5;
    public static double highGoalY = -22.5;
    public static double highGoalVelo = 5600; // 1; // roughly at a little over 12 volts
    public static double highGoalAngle = -22;

    // // TELE
    // power shot
    public static double powerShotAngle = 0; // angle the robot turns
    public static double powerShotVeloRight = 4000; // 0.72; //
    public static double powerShotVeloCenter = 4000; // 0.75; //
    public static double powerShotVeloLeft = 4000; // 0.72; //
    public static int shootCoolDown = 1000;

    // coordinates
    public static double powerShotX = -10; //X coord for the leftmost powershot
    public static double powerShotY = -40;   //Y coord for the leftmost powershot
    public static double pegDist = 8; // distance between each shot

    // // AUTO
    // power shot
    public static double autoPowerShotAngle = 0; // angle the robot turns
    public static double autoPowerShotVeloRight = 4000; // 0.72; //
    public static double autoPowerShotVeloCenter = 4000; // 0.75; //
    public static double autoPowerShotVeloLeft = 4000; // 0.72; //
    public static double rotFix = 1.2;

    // coordinates
    public static double autoRotFix = 1.2;
    public static double autoPowerShotX = -10;
    public static double autoPowerShotY = -38;
    public static double autoPegDist = 7; // distance between each shot

    public enum Position{
        LEFT, CENTER, RIGHT
    }

    public static Pose2d getPowerPose(double angle){
        return new Pose2d(powerShotX,powerShotY,angle);
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
                    Math.toRadians(autoPowerShotAngle + autoRotFix)
                );
                break;
            case LEFT: poseToReturn = new Pose2d(
                    autoPowerShotX,
                    autoPowerShotY + autoPegDist * 2,
                    Math.toRadians(autoPowerShotAngle + autoRotFix * 2)
                );
                break;
            default: poseToReturn = null;
                break;
        }
        return poseToReturn;
    }

    public static Pose2d autoGetPowerPose(double angle){
        return new Pose2d(autoPowerShotX, autoPowerShotY,angle);
    }
}
