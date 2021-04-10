package org.firstinspires.ftc.teamcode.auto.constants;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

public class AutoConstants {
    public enum Box {A,B,C}
    @Config
    public static class FirstBox {
        //These are the coordinates for the wobble goal placements.
        public static double dropAX = 6;
        public static double dropAY = -50;
        public static double dropAH = 90;

        public static double dropBX = 25;
        public static double dropBY = -27;
        public static double dropBH = 120;

        public static double dropCX = 48;
        public static double dropCY = -54;
        public static double dropCH = 120;


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
    @Config
    public static class SecondBox {
        //These are the coordinates for the secondary wobble goal placements.
        public static double dropAX = 6;
        public static double dropAY = -42;
        public static double dropAH = 90;

        public static double dropBX = 22;
        public static double dropBY = -18;
        public static double dropBH = 120;

        public static double dropCX = 46;
        public static double dropCY = -54;
        public static double dropCH = 120;

        public static double lineCX = 35;
        public static double lineCY = -44;
        public static double lineCH = 90;

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
        public static Pose2d getLinePose(){
            return new Pose2d(lineCX,lineCY,Math.toRadians(lineCH));
        }
    }
}
