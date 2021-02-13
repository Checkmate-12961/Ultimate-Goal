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
    public static double powerShotPower = .75;
    public static int shootCoolDown = 100;
    // coords
    public static double powerShotX = -14;
    public static double powerShotY = -10;
    public static double pegDist = 9.5; // distance between each shot
}
