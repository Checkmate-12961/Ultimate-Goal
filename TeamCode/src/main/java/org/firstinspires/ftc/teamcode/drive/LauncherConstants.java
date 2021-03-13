package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

@Config
public class LauncherConstants {
    public static int triggerActuationTime = 500;

    // high goal
    public static double highGoalX = -5;
    public static double highGoalY = -21.25;
    public static double highGoalVelo = 5200; // 1; // roughly at a little over 12 volts
    public static double highGoalAngle = -22;

    // // TELE
    // power shot
    public static double powerShotAngle = -10; // angle the robot turns
    public static double powerShotVeloRight = 3900; // 0.72; //
    public static double powerShotVeloCenter = 4100; // 0.75; //
    public static double powerShotVeloLeft = 4040; // 0.72; //
    public static int shootCoolDown = 1000;
    // coordinates
    public static double rotFix = 2.5 ;
    public static double powerShotX = -20; //X coord for the leftmost powershot
    public static double powerShotY = -27;   //Y coord for the leftmost powershot
    public static double pegDist = 4.5; // distance between each shot

    // // AUTO
    // power shot
    public static double autoPowerShotAngle = -12.5; // angle the robot turns
    public static double autoPowerShotVeloRight = 3900; // 0.72; //
    public static double autoPowerShotVeloCenter = 4200; // 0.75; //
    public static double autoPowerShotVeloLeft = 4000; // 0.72; //
    // coordinates
    public static double autoRotFix = 2.48;
    public static double autoPowerShotX = -9;
    public static double autoPowerShotY = -12;
    public static double autoPegDist = 6.65; // distance between each shot

    public static Vector2d getPowerVector(){
        return new Vector2d(powerShotX,powerShotY);
    }

    public static Pose2d getPowerPose(double angle){return new Pose2d(powerShotX,powerShotY,angle);}

    public static Pose2d autoGetPowerPose(double angle){return new Pose2d(autoPowerShotX, autoPowerShotY,angle);}
}
