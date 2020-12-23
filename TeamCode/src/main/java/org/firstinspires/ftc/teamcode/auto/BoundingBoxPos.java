package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;

@Config
public class BoundingBoxPos {
    // Simple class to allow changing the camera bounding box from the dash
    /*Guide to fix bounding box values
    Step 1: Connect the robot to a stream for instant feedback.
    Step 2: Move the TopLeft variables so that the top left corner of the bounding box is slightly higher and further left than the screen area that four rings can fit in.
    Step 3: Tune the Width and Height variables so that the bottom right corner of the bounding box is slightly lower and to the right of the screeb area that four rings can fit in.
    Step 4: Tune FourRingThresh to a value marginally lower than avg1 when four rings are on the field
    Step 5: Tune OneRingThresh to a value marginally lower than avg1 when one ring is on the field
     */
    public static int TopLeftX = 260;
    public static int TopLeftY = 130;
    public static int Width = 35;
    public static int Height = 45;
    public static int FourRingThresh = 150;
    public static int OneRingThresh = 135;
}
