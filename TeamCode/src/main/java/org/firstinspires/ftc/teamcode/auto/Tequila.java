package org.firstinspires.ftc.teamcode.auto;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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
public class Tequila extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private OpenCvWebcam webCam;
    private Vision.RingDeterminationPipeline.RingPosition ringPosSaved;

    Trajectory toLine;
    Trajectory rightShot;
    Trajectory midShot;
    Trajectory leftShot;
    Trajectory dropA;
    Trajectory dropB;
    Trajectory dropC;
    Trajectory toLineToo;

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

        Telemetry.Item runningItem = telemetry.addData("running","nothing");
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
        Vision.RingDeterminationPipeline pipeline = new Vision.RingDeterminationPipeline();
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
                .addDisplacementMarker(() -> {
                    runningItem.setValue("midShot");
                    telemetry.update();
                    drive.followTrajectoryAsync(midShot);
                })
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
                .addDisplacementMarker(() -> {
                    runningItem.setValue("leftShot");
                    telemetry.update();
                    drive.followTrajectoryAsync(leftShot);
                })
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
                .addDisplacementMarker(() -> {
                    runningItem.setValue("toLine");
                    telemetry.update();
                    drive.followTrajectoryAsync(toLine);
                })
                .build();

        onTrajBuild = nextTelemetry(onTrajBuild,trajBuildItem);
        //toLine moves the robot straight forward to the line
        nextTelemetry(onTrajBuild,trajBuildItem);
        toLine = drive.trajectoryBuilder(leftShot.end())
                .lineToSplineHeading(new Pose2d(LauncherMath.ApowerShotX, LauncherMath.ApowerShotY +LauncherMath.ApegDist *2+1, 0))
                .addDisplacementMarker(() -> {
                    if (ringPosSaved == Vision.RingDeterminationPipeline.RingPosition.NONE){
                        runningItem.setValue("dropA");
                        telemetry.update();
                        drive.followTrajectoryAsync(dropA);
                    } else if (ringPosSaved == Vision.RingDeterminationPipeline.RingPosition.ONE){
                        runningItem.setValue("dropB");
                        telemetry.update();
                        drive.followTrajectoryAsync(dropB);
                    } else if (ringPosSaved == Vision.RingDeterminationPipeline.RingPosition.FOUR){
                        runningItem.setValue("dropC");
                        telemetry.update();
                        drive.followTrajectoryAsync(dropC);
                    }
                })
                .build();

        onTrajBuild = nextTelemetry(onTrajBuild,trajBuildItem);

        dropA = drive.trajectoryBuilder(leftShot.end())
                .lineToSplineHeading(new Pose2d(AutoConstants.dropAX, AutoConstants.dropAY, AutoConstants.dropAH))
                .addDisplacementMarker(() -> {
                    depositWobble(drive);
                    runningItem.setValue("done");
                    telemetry.update();
                })
                .build();

        onTrajBuild = nextTelemetry(onTrajBuild,trajBuildItem);

        dropB = drive.trajectoryBuilder(leftShot.end())
                .lineToSplineHeading(new Pose2d(AutoConstants.dropBX, AutoConstants.dropBY, AutoConstants.dropBH))
                .addDisplacementMarker(() -> {
                    depositWobble(drive);
                    runningItem.setValue("done");
                    telemetry.update();
                })
                .build();

        onTrajBuild = nextTelemetry(onTrajBuild,trajBuildItem);

        dropC = drive.trajectoryBuilder(leftShot.end())
                .lineToSplineHeading(new Pose2d(AutoConstants.dropCX, AutoConstants.dropCY, AutoConstants.dropCH))
                .addDisplacementMarker(() -> {
                    depositWobble(drive);
                    runningItem.setValue("toLineToo");
                    telemetry.update();
                    drive.followTrajectoryAsync(toLineToo);
                })
                .build();

        onTrajBuild = nextTelemetry(onTrajBuild,trajBuildItem);

        toLineToo = drive.trajectoryBuilder(dropC.end())
                .lineToSplineHeading(new Pose2d(12, dropC.end().getY(),dropC.end().getHeading()))
                .addDisplacementMarker(() -> runningItem.setValue("done"))
                .build();

        nextTelemetry(onTrajBuild,trajBuildItem);

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
        onVal++;
        telemetryItem.setValue(onVal);
        telemetry.update();
        return onVal;
    }
    private void depositWobble(SampleMecanumDrive drive){
        drive.setWobblePosPow(-1,0);
        sleep(200);
        drive.setWobblePosPow(0,-.75); // arm is the power
        sleep(300); // milliseconds is the wait time
        drive.setWobblePosPow(0,0);
        sleep(300);
        drive.setWobblePosPow(1,0);
        sleep(200);
        drive.setWobblePosPow(1,.75);
        sleep(300);
    }
    private void retrieveArm(SampleMecanumDrive drive){

    }
}
