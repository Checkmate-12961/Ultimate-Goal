package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.Path;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Set of helper functions for drawing Road Runner paths and trajectories on dashboard canvases.
 */
public class DashboardUtil {
    private static final double DEFAULT_RESOLUTION = 2.0; // distance units; presumed inches
    private static final double ROBOT_RADIUS = 9; // in


    public static void drawPoseHistory(Canvas canvas, List<Pose2d> poseHistory) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];
        for (int i = 0; i < poseHistory.size(); i++) {
            Pose2d pose = poseHistory.get(i);
            xPoints[i] = pose.getX();
            yPoints[i] = pose.getY();
        }
        canvas.strokePolyline(xPoints, yPoints);
    }

    public static void drawSampledPath(Canvas canvas, Path path, double resolution) {
        int samples = (int) Math.ceil(path.length() / resolution);
        double[] xPoints = new double[samples];
        double[] yPoints = new double[samples];
        double dx = path.length() / (samples - 1);
        for (int i = 0; i < samples; i++) {
            double displacement = i * dx;
            Pose2d pose = path.get(displacement);
            xPoints[i] = pose.getX();
            yPoints[i] = pose.getY();
        }
        canvas.strokePolyline(xPoints, yPoints);
    }

    public static void drawSampledPath(Canvas canvas, Path path) {
        drawSampledPath(canvas, path, DEFAULT_RESOLUTION);
    }

    public static void drawRobot(Canvas canvas, Pose2d pose) {
        canvas.strokeCircle(pose.getX(), pose.getY(), ROBOT_RADIUS);
        Vector2d v = pose.headingVec().times(ROBOT_RADIUS);
        double x1 = pose.getX() + v.getX() / 2;
        double y1 = pose.getY() + v.getY() / 2;
        double x2 = pose.getX() + v.getX();
        double y2 = pose.getY() + v.getY();
        canvas.strokeLine(x1, y1, x2, y2);
    }

    /** 1--2
     *  |  |
     *  4--3
     */
    // For the record, I have no idea what is going on or if this will work at all.
    public static void drawRealisticRobot(Canvas canvas, Pose2d pose) {
        double r = Math.sqrt(2*Math.pow(ROBOT_RADIUS,2));
        double h1 = Math.PI * 0.75 + pose.getHeading();
        double h2 = Math.PI * 0.25 + pose.getHeading();
        double h3 = Math.PI * 1.25 + pose.getHeading();
        double h4 = Math.PI * 1.75 + pose.getHeading();
        // remember CiS (Cos + iSin)
        Vector2d v1 = new Vector2d(r * Math.cos(h1), r * Math.sin(h1)).plus(pose.vec());
        Vector2d v2 = new Vector2d(r * Math.cos(h2), r * Math.sin(h2)).plus(pose.vec());
        Vector2d v3 = new Vector2d(r * Math.cos(h3), r * Math.sin(h3)).plus(pose.vec());
        Vector2d v4 = new Vector2d(r * Math.cos(h4), r * Math.sin(h4)).plus(pose.vec());
        List<double[]> coordinatePairs = vectorsToCoordinatePairs(
                Arrays.asList(v1,v2,v3,v4)
            );
        canvas.strokePolygon(coordinatePairs.get(0), coordinatePairs.get(1));
        Vector2d v = pose.headingVec().times(ROBOT_RADIUS);
        double x1 = pose.getX() + v.getX() / 2;
        double y1 = pose.getY() + v.getY() / 2;
        double x2 = pose.getX() + v.getX();
        double y2 = pose.getY() + v.getY();
        canvas.strokeLine(x1, y1, x2, y2);
    }
    private static List<double[]> vectorsToCoordinatePairs (List<Vector2d> vectors) {
        double[] xList = new double[vectors.size()];
        double[] yList = new double[vectors.size()];
        for (int i = 0; i < vectors.size(); i++){
            xList[i] = vectors.get(i).getX();
            yList[i] = vectors.get(i).getY();
        }

        return Arrays.asList(
                xList,
                yList
            );
    }
}
