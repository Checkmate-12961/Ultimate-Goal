/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp
public class AutoWithVision extends LinearOpMode
{
    OpenCvWebcam webCam;
    RingDeterminationPipeline pipeline;


    @Override
    public void runOpMode()
    {
        // RR stuff
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-70.5, -20.25, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        drive.setWobblePosPow(0,0,0);

        // TODO: add in multiple paths for each of the different camera outputs
        int ringpos = pipeline.getAnalysis();
        telemetry.addData("RingPosGuess",ringpos);
        waitForStart();
        // traj0 and traj1 navigate the robot from the starting position to the goal
        Trajectory traj0 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-30,-24),0)
                .splineTo(new Vector2d(45,-36), Math.toRadians(180))
                .build();
        Trajectory traj1 = drive.trajectoryBuilder(traj0.end())
                .lineToSplineHeading(new Pose2d(64, -36, Math.toRadians(176)))
                .build();
        //traj2 pushes the robot slightly forward so the rings will be closer to the goal
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToSplineHeading(new Pose2d(68, -40 , Math.toRadians(174)))
                .build();
        //traj3 navigates the robot to the middle line
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .splineTo(new Vector2d(10,-20),Math.toRadians(174))
                .build();
        Trajectory misscircle = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-36, -60), Math.toRadians(180))
                .build();

        Trajectory droptraj;
        if(ringpos <= 135){
            droptraj = drive.trajectoryBuilder(startPose)
                    .splineTo(new Vector2d(-12,-60), Math.toRadians(0))
                    .build();
        }
        else if(ringpos <=150){
            droptraj = drive.trajectoryBuilder(startPose)
                    .splineTo(new Vector2d(36,-36), Math.toRadians(180))
                    .build();
        }
        else{
            droptraj = drive.trajectoryBuilder(startPose)
                    .splineTo(new Vector2d(60,-60), Math.toRadians(0))
                    .build();
        }
        Trajectory togoal = drive.trajectoryBuilder(droptraj.end())
                .lineToSplineHeading(new Pose2d(64, -36, Math.toRadians(176)))
                .build();
        Trajectory toline = drive.trajectoryBuilder(togoal.end())
                .splineTo(new Vector2d(36, -50), Math.toRadians(180))
                .build();

        // Camera stuff
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camra"), cameraMonitorViewId);
        pipeline = new RingDeterminationPipeline();
        webCam.setPipeline(pipeline);

        //listens for when the camera is opened
        webCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            public void onOpened()
            {
                //if the camera is open start steaming
                webCam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT  );
            }
        });



        if(isStopRequested()) return;
        drive.followTrajectory(misscircle);
        drive.followTrajectory(droptraj);
        drive.followTrajectory(togoal);

    }

    public static class RingDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the Ring position
         */
        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }


        //Some color constants
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        //The core values which define the location and size of the sample regions

        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(220,152);

        static final int REGION_WIDTH = 70;
        static final int REGION_HEIGHT = 25;

        final int FOUR_RING_THRESHOLD = 150;
        final int ONE_RING_THRESHOLD = 135;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        public volatile RingPosition position = RingPosition.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = RingPosition.FOUR; // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD){
                position = RingPosition.FOUR;
            }else if (avg1 > ONE_RING_THRESHOLD){
                position = RingPosition.ONE;
            }else{
                position = RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }
    }
}
