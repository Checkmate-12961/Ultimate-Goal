package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

@Config
public class LauncherMath {
    public static int triggerActuationTime = 500;

    // high goal
    public static double highGoalX = -1.25;
    public static double highGoalY = -22.25;
    public static double highGoalVelo = 5200; // 1; // roughly at a little over 12 volts
    public static double highGoalAngle = -Math.PI/12;

    // // TELE
    // power shot
    public static double powerShotAngle = 0; // angle the robot turns
    public static double powerShotVeloRight = 3900; // 0.72; //
    public static double powerShotVeloCenter = 4200; // 0.75; //
    public static double powerShotVeloLeft = 4060; // 0.72; //
    public static int shootCoolDown = 1000;
    // coordinates
    public static double rotFix = 2.5 ;
    public static double powerShotX = -20; //X coord for the leftmost powershot
    public static double powerShotY = 2;   //Y coord for the leftmost powershot
    public static double pegDist = 6.7; // distance between each shot

    // // AUTO
    // power shot
    public static double ApowerShotAngle = -12.5; // angle the robot turns
    public static double ApowerShotVeloRight = 3900; // 0.72; //
    public static double ApowerShotVeloCenter = 4200; // 0.75; //
    public static double ApowerShotVeloLeft = 4000; // 0.72; //
    // coordinates
    public static double ArotFix = 2.48;
    public static double ApowerShotX = -9;
    public static double ApowerShotY = -12;
    public static double ApegDist = 6.65; // distance between each shot

    public static Vector2d getPowerVector(){
        return new Vector2d(powerShotX,powerShotY);
    }

    public static Pose2d getPowerPose(double angle){return new Pose2d(powerShotX,powerShotY,angle);}

    public static Pose2d AgetPowerPose(double angle){return new Pose2d(ApowerShotX,ApowerShotY,angle);}
}
