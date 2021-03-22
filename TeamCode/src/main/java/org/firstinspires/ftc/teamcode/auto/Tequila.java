package org.firstinspires.ftc.teamcode.auto;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.LauncherConstants;
import org.firstinspires.ftc.teamcode.drive.PoseUtils;
import org.firstinspires.ftc.teamcode.drive.HungryHippoDrive;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(group = "Alcohol")
public class Tequila extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private OpenCvWebcam webCam;
    private VisionHelper.RingDeterminationPipeline.RingPosition ringPosSaved;

    @SuppressWarnings("FieldCanBeLocal")
    private Trajectory rightShot;
    private Trajectory midShot;
    private Trajectory leftShot;
    private Trajectory dropA;
    private Trajectory dropB;
    private Trajectory dropC;
    private Trajectory toLineToo;

    private Telemetry.Item trajBuildItem;
    private Telemetry.Item runningItem;

    private HungryHippoDrive drive;

    private int onTrajBuild = 0;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry.setAutoClear(false);
        Telemetry.Item initItem = telemetry.addData("Initializing...","Setting up hardware");
        telemetry.update();

        // RR stuff
        drive = new HungryHippoDrive(hardwareMap);
        PoseUtils.currentPose = PoseUtils.globalStartPose;
        Pose2d startPose = PoseUtils.currentPose;
        drive.setPoseEstimate(startPose);

        runningItem = telemetry.addData("running","nothing");
        Telemetry.Item xItem = telemetry.addData("x",drive.getPoseEstimate().getX());
        Telemetry.Item yItem = telemetry.addData("y",drive.getPoseEstimate().getY());
        Telemetry.Item headingItem = telemetry.addData("θ",drive.getPoseEstimate().getHeading());

        initItem.setValue("Resetting servos");
        telemetry.update();
        drive.setWobblePosPow(1,0);

        initItem.setValue("Starting camera feed");
        telemetry.update();
        // Camera stuff
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camra"), cameraMonitorViewId);
        VisionHelper.RingDeterminationPipeline pipeline = new VisionHelper.RingDeterminationPipeline();
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

        trajBuildItem = telemetry.addData("Built", onTrajBuild);
        telemetry.update();


        rightShot = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(LauncherConstants.autoGetPowerPose(LauncherConstants.Position.RIGHT))
                .addDisplacementMarker(() -> {
                    sleep(LauncherConstants.shootCoolDown*2);
                    drive.pressTrigger(true);
                    sleep(LauncherConstants.triggerActuationTime);
                    drive.pressTrigger(false);
                    drive.revFlywheel(-LauncherConstants.autoPowerShotVeloCenter);
                })
                .addDisplacementMarker(() -> runTrajectory(midShot))
                .build();

        nextTelemetry();
        midShot = drive.trajectoryBuilder(rightShot.end())
                .lineToSplineHeading(LauncherConstants.autoGetPowerPose(LauncherConstants.Position.CENTER))
                .addDisplacementMarker(() -> {
                    sleep(LauncherConstants.shootCoolDown);
                    drive.pressTrigger(true);
                    sleep(LauncherConstants.triggerActuationTime);
                    drive.pressTrigger(false);
                    drive.revFlywheel(-LauncherConstants.autoPowerShotVeloLeft);
                })
                .addDisplacementMarker(() -> runTrajectory(leftShot))
                .build();

        nextTelemetry();
        leftShot = drive.trajectoryBuilder(midShot.end())
                .lineToSplineHeading(LauncherConstants.autoGetPowerPose(LauncherConstants.Position.LEFT))
                .addDisplacementMarker(() -> {
                    sleep(LauncherConstants.shootCoolDown);
                    drive.pressTrigger(true);
                    sleep(LauncherConstants.triggerActuationTime);
                    drive.pressTrigger(false);
                    drive.revFlywheel(0);
                })
                .addDisplacementMarker(() -> {
                    Trajectory toFollow;
                    switch (ringPosSaved){
                        case NONE:
                            toFollow = dropA;
                            break;
                        case ONE:
                            toFollow = dropB;
                            break;
                        case FOUR:
                            toFollow = dropC;
                            break;
                        default:
                            toFollow = null;
                            break;
                    }
                    runTrajectory(toFollow);
                })
                .build();

        nextTelemetry();
        dropA = drive.trajectoryBuilder(leftShot.end())
                .lineToSplineHeading(new Pose2d(AutoConstants.dropAX, AutoConstants.dropAY, AutoConstants.dropAH))
                .addDisplacementMarker(() -> {
                    try {
                        MoveWobble.depositWobble(drive);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    runningItem.setValue("done");
                    telemetry.update();
                })
                .build();

        nextTelemetry();

        dropB = drive.trajectoryBuilder(leftShot.end())
                .lineToSplineHeading(new Pose2d(AutoConstants.dropBX, AutoConstants.dropBY, AutoConstants.dropBH))
                .addDisplacementMarker(() -> {
                    try {
                        MoveWobble.depositWobble(drive);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    runningItem.setValue("done");
                    telemetry.update();
                })
                .build();

        nextTelemetry();

        dropC = drive.trajectoryBuilder(leftShot.end())
                .lineToSplineHeading(new Pose2d(AutoConstants.dropCX, AutoConstants.dropCY, AutoConstants.dropCH))
                .addDisplacementMarker(() -> {
                    try {
                        MoveWobble.depositWobble(drive);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    runTrajectory(toLineToo);
                })
                .build();

        nextTelemetry();

        toLineToo = drive.trajectoryBuilder(dropC.end())
                .lineToSplineHeading(new Pose2d(12, dropC.end().getY(),dropC.end().getHeading()))
                .addDisplacementMarker(() -> runningItem.setValue("done"))
                .build();

        nextTelemetry();

        telemetry.removeItem(trajBuildItem);
        Telemetry.Item ringPosEst = telemetry.addData("RingPosEst", pipeline.getPosition());
        Telemetry.Item ringAnal = telemetry.addData("RingAnalysis", pipeline.getAnalysis());
        initItem.setValue(String.format("Done. Took %f milliseconds",runtime.milliseconds()));
        telemetry.update();

        waitForStart();

        ringPosSaved = pipeline.getPosition();
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
        runningItem.setValue("rightShot");
        telemetry.update();
        drive.revFlywheel(-LauncherConstants.autoPowerShotVeloRight);

        int ticks = 0;
        Telemetry.Item avgTPS = telemetry.addData("AvgTPS", ticks / (runtime.seconds()-initTime/1000));

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            runtimeItem.setValue(
                    String.format(
                            "%fms",
                            runtime.milliseconds()-initTime
                    ));
            Pose2d tempPose = drive.getPoseEstimate();
            xItem.setValue(PoseUtils.currentPose.getX());
            yItem.setValue(PoseUtils.currentPose.getY());

            ringPosEst.setValue(pipeline.getPosition());
            ringAnal.setValue(pipeline.getAnalysis());

            headingItem.setValue(tempPose.getHeading());

            ticks += 1;
            avgTPS.setValue(ticks / (runtime.seconds()-initTime/1000));

            if (Math.abs(PoseUtils.currentPose.getX() - tempPose.getX()) < 70 && Math.abs(PoseUtils.currentPose.getY() - tempPose.getY()) < 70) {
                PoseUtils.currentPose = tempPose;
            }
            telemetry.update();
        }
    }

    private void nextTelemetry(){
        onTrajBuild++;
        trajBuildItem.setValue(onTrajBuild);
        telemetry.update();
    }
    private void runTrajectory(Trajectory toRun){
        runningItem.setValue(toRun);
        telemetry.update();
        drive.followTrajectoryAsync(toRun);
    }
}
