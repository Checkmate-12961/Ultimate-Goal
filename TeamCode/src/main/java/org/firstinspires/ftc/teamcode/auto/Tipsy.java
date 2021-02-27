package org.firstinspires.ftc.teamcode.auto;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
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

@Config
@Autonomous(group = "Alcohol")
public class Tipsy extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    OpenCvWebcam webCam;
    Vision.RingDeterminationPipeline pipeline;
    private Vision.RingDeterminationPipeline.RingPosition ringPosSaved;

    Trajectory toLine, dropA,dropB,dropC,toLineToo;

    public static double dropAX = 16;
    public static double dropAY = -32;
    public static double dropAH = Math.PI/2;

    public static double dropBX = 18;
    public static double dropBY = -18;
    public static double dropBH = Math.PI*.75;

    public static double dropCX = 46;
    public static double dropCY = -36;
    public static double dropCH = Math.PI*.75;

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
        drive.setWobblePosPow(-1,0);

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

        initItem.setValue("Building trajectories");

        drive.dashboard.startCameraStream(webCam, 10);

        int onTrajBuild = 0;
        Telemetry.Item trajBuildItem = telemetry.addData("Built", onTrajBuild);
        telemetry.update();

        toLine = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(LauncherMath.ApowerShotX, LauncherMath.ApowerShotY +LauncherMath.ApegDist *2, Math.toRadians(LauncherMath.ApowerShotAngle+LauncherMath.ArotFix*2)))
                .addDisplacementMarker(() -> {
                    if (ringPosSaved == Vision.RingDeterminationPipeline.RingPosition.NONE){
                        drive.followTrajectoryAsync(dropA);
                    } else if (ringPosSaved == Vision.RingDeterminationPipeline.RingPosition.ONE){
                        drive.followTrajectoryAsync(dropB);
                    } else if (ringPosSaved == Vision.RingDeterminationPipeline.RingPosition.FOUR){
                        drive.followTrajectoryAsync(dropC);
                    }
                })
                .build();

        onTrajBuild = nextTelemetry(onTrajBuild,trajBuildItem);

        dropA = drive.trajectoryBuilder(toLine.end())
                .lineToSplineHeading(new Pose2d(dropAX,dropAY,dropAH))
                .build();

        onTrajBuild = nextTelemetry(onTrajBuild,trajBuildItem);

        dropB = drive.trajectoryBuilder(toLine.end())
                .lineToSplineHeading(new Pose2d(dropBX,dropBY,dropBH))
                .build();

        onTrajBuild = nextTelemetry(onTrajBuild,trajBuildItem);

        dropC = drive.trajectoryBuilder(toLine.end())
                .lineToSplineHeading(new Pose2d(dropCX,dropCY,dropCH))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(toLineToo))
                .build();

        onTrajBuild = nextTelemetry(onTrajBuild,trajBuildItem);

        toLineToo = drive.trajectoryBuilder(dropC.end())
                .lineToSplineHeading(new Pose2d(12, dropC.end().getY(),dropC.end().getHeading()))
                .build();

        nextTelemetry(onTrajBuild,trajBuildItem);

        telemetry.removeItem(trajBuildItem);
        initItem.setValue(String.format("Done. Took %f ms",runtime.milliseconds()));
        telemetry.update();

        waitForStart();

        ringPosSaved = pipeline.getPosition();
        Telemetry.Item ringPosEst = telemetry.addData("RingPosEst", ringPosSaved);
        Telemetry.Item ringAnal = telemetry.addData("RingAnal", pipeline.getAnalysis());
        telemetry.update();

        if(isStopRequested()) return;
        telemetry.removeItem(initItem);
        double initTime = runtime.milliseconds();

        drive.followTrajectoryAsync(toLine);

        Telemetry.Item runtimeItem = telemetry.addData(
                "Runtime",
                String.format(
                        "%fms",
                        runtime.milliseconds()-initTime
                ));
        telemetry.update();

        drive.revFlywheel(0);

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

            ringPosEst.setValue(ringPosSaved);
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
        sleep(200);
        drive.setWobblePosPow(0,.75); // arm is the power
        sleep(300); // milliseconds is the wait time
        drive.setWobblePosPow(0,0);
        sleep(300);
        drive.setWobblePosPow(1,0);
        sleep(200);
    }
}
