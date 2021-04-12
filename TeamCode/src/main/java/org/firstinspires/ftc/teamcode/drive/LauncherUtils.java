package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.drive.launcherConstants.AutoPowerConstants;
import org.firstinspires.ftc.teamcode.drive.launcherConstants.EndgamePowerConstants;
import org.firstinspires.ftc.teamcode.drive.launcherConstants.HighGoalConstants;

@Config
public class LauncherUtils {
    public static int triggerActuationTime = 500;

    // everything
    public static double flywheelThreshold = 10; // In RPM

    public enum Position{LEFT, CENTER, RIGHT}

    public static Pose2d getPowerPose(Position pos){
        Pose2d poseToReturn;
        switch (pos){
            case RIGHT: poseToReturn = new Pose2d(
                    EndgamePowerConstants.X,
                    EndgamePowerConstants.YRight,
                    Math.toRadians(EndgamePowerConstants.angle)
            );
                break;
            case CENTER: poseToReturn = new Pose2d(
                    EndgamePowerConstants.X,
                    EndgamePowerConstants.YCenter,
                    Math.toRadians(EndgamePowerConstants.angle)
            );
                break;
            case LEFT: poseToReturn = new Pose2d(
                    EndgamePowerConstants.X,
                    EndgamePowerConstants.YLeft,
                    Math.toRadians(EndgamePowerConstants.angle)
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
                    AutoPowerConstants.X,
                    AutoPowerConstants.YRight,
                    Math.toRadians(AutoPowerConstants.angle)
                );
                break;
            case CENTER: poseToReturn = new Pose2d(
                    AutoPowerConstants.X,
                    AutoPowerConstants.YCenter,
                    Math.toRadians(AutoPowerConstants.angle)
                );
                break;
            case LEFT: poseToReturn = new Pose2d(
                    AutoPowerConstants.X,
                    AutoPowerConstants.YLeft,
                    Math.toRadians(AutoPowerConstants.angle)
                );
                break;
            default: poseToReturn = null;
                break;
        }
        return poseToReturn;
    }

    public static Pose2d getHighGoalPose(){
        return new Pose2d(HighGoalConstants.X,HighGoalConstants.Y,HighGoalConstants.angle);
    }
}
