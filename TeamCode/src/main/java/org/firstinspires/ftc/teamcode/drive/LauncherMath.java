package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;

@Config
public class LauncherMath {
    public static int triggerActuationTime = 500;

    // high goal
    public static double highGoalX = -7;
    public static double highGoalY = -31;
    public static double highGoalPower = 1;
    public static double highGoalAngle = -15;

    // power shot
    public static double powerShotAngle = 0; // angle the robo turns
    public static double powerShotPowerRight = 0.72;
    public static double powerShotPowerCenter = 0.74;
    public static double powerShotPowerLeft = 0.72;
    public static int shootCoolDown = 1000;
    // coords
    public static double rotFix = 2.4;
    public static double powerShotX = -12;
    public static double powerShotY = -9.2;
    public static double pegDist = 6.6; // distance between each shot
}
