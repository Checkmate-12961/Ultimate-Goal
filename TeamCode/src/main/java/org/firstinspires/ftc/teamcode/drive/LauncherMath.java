package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

@Config
public class LauncherMath {
    public static int triggerActuationTime = 500;

    // high goal
    public static double highGoalX = -7;
    public static double highGoalY = -31;
    public static double highGoalVelo = 5200; // 1; // roughly at a little over 12 volts
    public static double highGoalAngle = -15;

    // power shot
    public static double powerShotAngle = 0; // angle the robot turns
    public static double powerShotVeloRight = 4060; // 0.72; //
    public static double powerShotVeloCenter = 4250; // 0.75; //
    public static double powerShotVeloLeft = 4060; // 0.72; //
    public static int shootCoolDown = 1000;
    // coordinates
    public static double rotFix = 2.5;
    public static double powerShotX = -12;
    public static double powerShotY = -9.25;
    public static double pegDist = 6.65; // distance between each shot

    public static Vector2d getPowerVector(){
        return new Vector2d(powerShotX,powerShotY);
    }

    public static Pose2d getPowerPose(double angle){
        return new Pose2d(powerShotX,powerShotY,angle);
    }
}
