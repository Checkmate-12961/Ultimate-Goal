package org.firstinspires.ftc.teamcode.auto;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class Robert extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    OpenCvWebcam webCam;
    Vision.RingDeterminationPipeline pipeline;

    Trajectory clearance;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setAutoClear(false);
        Telemetry.Item initItem = telemetry.addData("Initializing...", "Setting up hardware");
        telemetry.update();

        PoseStorage.currentPose = new Pose2d(-62, -21, Math.toRadians(0));

        // RR stuff
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = PoseStorage.currentPose;
        drive.setPoseEstimate(startPose);

        Telemetry.Item xItem = telemetry.addData("x", drive.getPoseEstimate().getX());
        Telemetry.Item yItem = telemetry.addData("y", drive.getPoseEstimate().getY());
        Telemetry.Item headingItem = telemetry.addData("θ", drive.getPoseEstimate().getHeading());

        int onTrajBuild = 1;
        telemetry.addData("Building", onTrajBuild);
        telemetry.update();

        initItem.setValue("Starting camera feed");
        telemetry.update();
        // Camera stuff
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camra"), cameraMonitorViewId);
        pipeline = new Vision.RingDeterminationPipeline();
        webCam.setPipeline(pipeline);

        //listens for when the camera is opened
        webCam.openCameraDeviceAsync(() -> {
            //if the camera is open start steaming
            webCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        });

        initItem.setValue("Checking ring position");
        telemetry.update();
        telemetry.addData("RingPosGuess", pipeline.getPosition());

        initItem.setValue("Building trajectories");


        waitForStart();

        if (isStopRequested()) return;
        telemetry.removeItem(initItem);
        double initTime = runtime.milliseconds();

        Telemetry.Item runtimeItem = telemetry.addData(
                "Runtime",
                String.format(
                        "%fms",
                        runtime.milliseconds() - initTime
                ));
        telemetry.update();

        drive.setWobblePosPow(0, 1);
        sleep(1000);
        drive.setWobblePosPow(-1,0);
        sleep(1000);
        drive.setWobblePosPow(0,-1);
        sleep(1000);

        drive.followTrajectoryAsync(clearance);

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            runtimeItem.setValue(
                    String.format(
                            "%fms",
                            runtime.milliseconds() - initTime
                    ));
            Pose2d tempPose = drive.getPoseEstimate();
            xItem.setValue(tempPose.getX());
            yItem.setValue(tempPose.getY());
            headingItem.setValue(tempPose.getHeading());
            telemetry.update();
        }
    }

}
