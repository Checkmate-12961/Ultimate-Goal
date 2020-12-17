package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;

@Config
public class LauncherMath {
    public static double ringVelo = 0; // TODO: Find the velocity of rings exiting the launcher

    public double calcAngle(double dist, double height, double velocity){
        return 0;
    }
    public double calcAngle(double dist, double height){
        return calcAngle(dist, height, ringVelo);
    }
}
