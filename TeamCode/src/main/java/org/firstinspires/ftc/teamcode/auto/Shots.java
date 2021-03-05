package org.firstinspires.ftc.teamcode.auto;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.LauncherMath;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(group = "Alcohol")
public class Shots extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    OpenCvWebcam webCam;
    Vision.RingDeterminationPipeline pipeline;

    Trajectory toLine, rightShot, leftShot, midShot;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry.setAutoClear(false);
        Telemetry.Item initItem = telemetry.addData("Initializing...","Setting up hardware");
        telemetry.update();

        // RR stuff
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        PoseStorage.currentPose = new Pose2d(-62, -19.5 , Math.toRadians(0) );
        Pose2d startPose = PoseStorage.currentPose;
        drive.setPoseEstimate(startPose);

        Telemetry.Item xItem = telemetry.addData("x",drive.getPoseEstimate().getX());
        Telemetry.Item yItem = telemetry.addData("y",drive.getPoseEstimate().getY());
        Telemetry.Item headingItem = telemetry.addData("θ",drive.getPoseEstimate().getHeading());

        initItem.setValue("Resetting servos");
        telemetry.update();
        drive.setWobblePosPow(0,0);

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
            webCam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT  );
        });

        initItem.setValue("Checking ring position");
        telemetry.update();

        initItem.setValue("Building trajectories");

        drive.dashboard.startCameraStream(webCam, 10);

        int onTrajBuild = 0;
        Telemetry.Item trajBuildItem = telemetry.addData("Built", onTrajBuild);
        telemetry.update();

        rightShot = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(LauncherMath.AgetPowerPose(Math.toRadians(LauncherMath.ApowerShotAngle)))
                .addDisplacementMarker(() -> {
                    sleep(LauncherMath.shootCoolDown*2);
                    drive.pressTrigger(true);
                    sleep(LauncherMath.triggerActuationTime);
                    drive.pressTrigger(false);
                    drive.revFlywheel(-LauncherMath.ApowerShotVeloCenter);
                })
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(midShot))
                .build();

        onTrajBuild = nextTelemetry(onTrajBuild,trajBuildItem);
        midShot = drive.trajectoryBuilder(rightShot.end())
                .lineToSplineHeading(new Pose2d(LauncherMath.ApowerShotX, LauncherMath.ApowerShotY +LauncherMath.ApegDist, Math.toRadians(LauncherMath.ApowerShotAngle+LauncherMath.ArotFix)))
                .addDisplacementMarker(() -> {
                    sleep(LauncherMath.shootCoolDown);
                    drive.pressTrigger(true);
                    sleep(LauncherMath.triggerActuationTime);
                    drive.pressTrigger(false);
                    drive.revFlywheel(-LauncherMath.ApowerShotVeloLeft);
                })
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(leftShot))
                .build();

        onTrajBuild = nextTelemetry(onTrajBuild,trajBuildItem);
        leftShot = drive.trajectoryBuilder(midShot.end())
                .lineToSplineHeading(new Pose2d(LauncherMath.ApowerShotX, LauncherMath.ApowerShotY +LauncherMath.ApegDist *2, Math.toRadians(LauncherMath.ApowerShotAngle+LauncherMath.ArotFix*2)))
                .addDisplacementMarker(() -> {
                    sleep(LauncherMath.shootCoolDown);
                    drive.pressTrigger(true);
                    sleep(LauncherMath.triggerActuationTime);
                    drive.pressTrigger(false);
                    drive.revFlywheel(0);
                })
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(toLine))
                .build();

        onTrajBuild = nextTelemetry(onTrajBuild,trajBuildItem);
        //toLine moves the robot straight forward to the line
        toLine = drive.trajectoryBuilder(leftShot.end())
                .splineTo(new Vector2d(12, LauncherMath.ApowerShotY + 2*LauncherMath.ApegDist), Math.toRadians(0))
                .build();
        nextTelemetry(onTrajBuild,trajBuildItem);

        telemetry.removeItem(trajBuildItem);
        Telemetry.Item ringPosEst = telemetry.addData("RingPosEst", pipeline.getPosition());
        Telemetry.Item ringAnal = telemetry.addData("RingAnalysis", pipeline.getAnalysis());
        initItem.setValue(String.format("Done. Took %f milliseconds",runtime.milliseconds()));
        telemetry.update();

        waitForStart();

        if(isStopRequested()) return;
        telemetry.removeItem(initItem);
        double initTime = runtime.milliseconds();

        drive.followTrajectoryAsync(rightShot);

        Telemetry.Item runtimeItem = telemetry.addData(
                "Runtime",
                String.format(
                        "%fms",
                        runtime.milliseconds()-initTime
                ));
        telemetry.update();

        drive.revFlywheel(-LauncherMath.ApowerShotVeloRight);

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

            ringPosEst.setValue(pipeline.getPosition());
            ringAnal.setValue(pipeline.getAnalysis());

            headingItem.setValue(tempPose.getHeading());
            telemetry.update();
        }
        PoseStorage.currentPose = drive.getPoseEstimate();
    }

    private int nextTelemetry(int onVal, Telemetry.Item telemetryItem){
        int newOnVal = onVal +1;
        telemetryItem.setValue(newOnVal);
        telemetry.update();
        return newOnVal;
    }
}
