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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.HungryHippoDrive;
import org.firstinspires.ftc.teamcode.drive.PoseUtils;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Locale;

@SuppressWarnings("unused")
@Autonomous
@Disabled
public class LinePark extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private OpenCvWebcam webCam;

    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry.setAutoClear(false);
        Telemetry.Item initItem = telemetry.addData("Initializing...","Setting up hardware");
        telemetry.update();

        // RR stuff
        HungryHippoDrive drive = new HungryHippoDrive(hardwareMap);
        Pose2d startPose = PoseUtils.currentPose;
        drive.setPoseEstimate(startPose);

        Telemetry.Item xItem = telemetry.addData("x",drive.getPoseEstimate().getX());
        Telemetry.Item yItem = telemetry.addData("y",drive.getPoseEstimate().getY());
        Telemetry.Item headingItem = telemetry.addData("θ",drive.getPoseEstimate().getHeading());

        initItem.setValue("Resetting servos");
        telemetry.update();

        initItem.setValue("Starting camera feed");
        telemetry.update();
        // Camera stuff
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, HungryHippoDrive.WEBCAM_NAME), cameraMonitorViewId);
        VisionHelper.RingDeterminationPipeline pipeline = new VisionHelper.RingDeterminationPipeline();
        webCam.setPipeline(pipeline);

        //listens for when the camera is opened
        webCam.openCameraDeviceAsync(() -> {
            //if the camera is open start steaming
            webCam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT  );
        });

        initItem.setValue("Checking ring position");
        telemetry.update();

        // DONE: add in multiple paths for each of the different camera outputs
        int ringPos = pipeline.getAnalysis();
        initItem.setValue("Building trajectories");
        telemetry.addData("RingPosGuess",ringPos);

        int onTrajBuild = 0;
        Telemetry.Item trajBuildItem = telemetry.addData("Built", onTrajBuild);
        telemetry.update();


        //toLine moves the robot straight forward to the line
        Trajectory toLine = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(12, -20.25), Math.toRadians(0))
                .build();

        onTrajBuild = nextTelemetry(onTrajBuild,trajBuildItem);


        nextTelemetry(onTrajBuild,trajBuildItem);

        // add more trajectories here if needed

        telemetry.removeItem(trajBuildItem);
        initItem.setValue(String.format(Locale.ENGLISH,"Done. Took %f milliseconds",runtime.milliseconds()));
        telemetry.update();

        waitForStart();

        if(isStopRequested()) return;
        telemetry.removeItem(initItem);
        double initTime = runtime.milliseconds();

        drive.followTrajectoryAsync(toLine);

        Telemetry.Item runtimeItem = telemetry.addData(
                "Runtime",
                String.format(Locale.ENGLISH, "%fms", runtime.milliseconds() - initTime));
        telemetry.update();

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            runtimeItem.setValue(
                    String.format(Locale.ENGLISH, "%fms", runtime.milliseconds() - initTime));
            Pose2d tempPose = drive.getPoseEstimate();
            xItem.setValue(tempPose.getX());
            yItem.setValue(tempPose.getY());
            headingItem.setValue(tempPose.getHeading());
            telemetry.update();
        }

        PoseUtils.currentPose = drive.getPoseEstimate();
    }

    private int nextTelemetry(int onVal,Telemetry.Item telemetryItem){
        int newOnVal = onVal + 1;
        telemetryItem.setValue(newOnVal);
        telemetry.update();
        return newOnVal;
    }
}
