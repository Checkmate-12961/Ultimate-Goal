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

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class PlanA extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    OpenCvWebcam webCam;

    Trajectory clearance,flip,toGoal,toLine;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry.setAutoClear(false);
        Telemetry.Item initItem = telemetry.addData("Initializing...","Setting up hardware");
        telemetry.update();

        PoseStorage.currentPose = new Pose2d(-70.5, -20.25 , Math.toRadians(0));

        // RR stuff
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = PoseStorage.currentPose;
        drive.setPoseEstimate(startPose);

        Telemetry.Item xItem = telemetry.addData("x",drive.getPoseEstimate().getX());
        Telemetry.Item yItem = telemetry.addData("y",drive.getPoseEstimate().getY());
        Telemetry.Item headingItem = telemetry.addData("θ",drive.getPoseEstimate().getHeading());

        initItem.setValue("Resetting servos");
        telemetry.update();
        drive.setWobblePosPow(0,0,0);

        //listens for when the camera is opened
        webCam.openCameraDeviceAsync(() -> {
            //if the camera is open start steaming
            webCam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT  );
        });

        initItem.setValue("Checking ring position");
        telemetry.update();

        int onTrajBuild = 0;
        Telemetry.Item trajBuildItem = telemetry.addData("Built", onTrajBuild);
        telemetry.update();


        //clearance moves the robot forward almost to the center line, so that it will miss the circle stack and be able to turn without hitting walls.
        clearance = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(0, -20, Math.toRadians(0)))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(flip))
                .build();

        onTrajBuild = nextTelemetry(onTrajBuild,trajBuildItem);


        //flip flips the robot 180 degrees, lest it hit the wall when it moves to the goal
        flip = drive.trajectoryBuilder(clearance.end())
                .lineToSplineHeading(new Pose2d(30,-20, Math.toRadians((180))))
                .addDisplacementMarker(() -> {
                    drive.setWobblePosPow(0, 1, 0);
                    sleep(1000);
                    drive.setWobblePosPow(1,0,0);
                    sleep(1000);
                    drive.setWobblePosPow(0,-1,0);
                    sleep(1000);
                })
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(toGoal))
                .build();

        onTrajBuild = nextTelemetry(onTrajBuild,trajBuildItem);

        //toGoal moves the robot before the goal, so that it may deposit circles into it.
        toGoal = drive.trajectoryBuilder(flip.end())
                .lineToSplineHeading(new Pose2d(90, -40, Math.toRadians(180)))
                .addDisplacementMarker(() -> {
                    drive.setIntakePowers(0,-1,-1);
                    sleep(3000);
                    drive.setIntakePowers(0,0,0);
                })
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(toLine))
                .build();

        onTrajBuild = nextTelemetry(onTrajBuild,trajBuildItem);

        toLine = drive.trajectoryBuilder(toGoal.end())
                .lineToSplineHeading(new Pose2d(24, -50, Math.toRadians(180)))
                .build();

        nextTelemetry(onTrajBuild,trajBuildItem);

        // add more trajectories here if needed

        telemetry.removeItem(trajBuildItem);
        initItem.setValue(String.format("Done. Took %f milliseconds",runtime.milliseconds()));
        telemetry.update();

        waitForStart();

        if(isStopRequested()) return;
        telemetry.removeItem(initItem);
        double initTime = runtime.milliseconds();

        Telemetry.Item runtimeItem = telemetry.addData(
                "Runtime",
                String.format(
                        "%fms",
                        runtime.milliseconds()-initTime
                ));
        telemetry.update();

        drive.setWobblePosPow(0, 1, 0);
        sleep(1000);
        drive.setWobblePosPow(-1,0,0);
        sleep(1000);
        drive.setWobblePosPow(0,-1,0);
        sleep(1000);

        drive.followTrajectoryAsync(clearance);

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            runtimeItem.setValue(
                    String.format(
                            "%fms",
                            runtime.milliseconds()-initTime
                    ));
            Pose2d tempPose = drive.getPoseEstimate();
            xItem.setValue(tempPose.getX());
            yItem.setValue(tempPose.getY());
            headingItem.setValue(tempPose.getHeading());
            telemetry.update();
        }

        PoseStorage.currentPose = drive.getPoseEstimate();
    }
    private int nextTelemetry(int onVal,Telemetry.Item telemetryItem){
        onVal++;
        telemetryItem.setValue(onVal);
        telemetry.update();
        return onVal;
    }
}
