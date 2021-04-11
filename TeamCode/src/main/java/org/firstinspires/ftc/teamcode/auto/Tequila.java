package org.firstinspires.ftc.teamcode.auto;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.drive.DrunkenHippoDrive;
import org.firstinspires.ftc.teamcode.drive.LauncherUtils;
import org.firstinspires.ftc.teamcode.drive.PoseUtils;
import org.firstinspires.ftc.teamcode.drive.launcherConstants.AutoPowerConstants;

@Autonomous(group = "Alcohol")
public class Tequila extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private DrunkenHippoDrive.RingPosition ringPosSaved;

    @SuppressWarnings("FieldCanBeLocal")
    private Trajectory missRings;
    private Trajectory rightShot;
    private Trajectory midShot;
    private Trajectory leftShot;
    private Trajectory dropA;
    private Trajectory dropB;
    private Trajectory dropC;
    private Trajectory toLineC;

    private Telemetry.Item trajBuildItem;
    private Telemetry.Item runningItem;

    private DrunkenHippoDrive drive;

    private int onTrajBuild = 0;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry.setAutoClear(false);
        Telemetry.Item initItem = telemetry.addData("Initializing...","Setting up hardware");
        telemetry.update();

        // RR stuff
        drive = new DrunkenHippoDrive(hardwareMap);
        PoseUtils.currentPose = PoseUtils.getStartPose();
        Pose2d startPose = PoseUtils.currentPose;
        drive.setPoseEstimate(startPose);

        runningItem = telemetry.addData("running","nothing");
        Telemetry.Item xItem = telemetry.addData("x",drive.getPoseEstimate().getX());
        Telemetry.Item yItem = telemetry.addData("y",drive.getPoseEstimate().getY());
        Telemetry.Item headingItem = telemetry.addData("θ",drive.getPoseEstimate().getHeading());

        initItem.setValue("Checking ring position");
        telemetry.update();

        ringPosSaved = drive.getPosition();
        telemetry.addData("RingPos", ringPosSaved);
        Telemetry.Item ringAnal = telemetry.addData("RingAnalNow", drive.getAnalysis());

        initItem.setValue("Building trajectories");

        // You can uncomment this for troubleshooting the camera
        // Streams the camera to the dash
        // drive.dashboard.startCameraStream(webCam, 10);

        trajBuildItem = telemetry.addData("Built", onTrajBuild);
        telemetry.update();

        missRings = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(0, startPose.getY(), 0))
                .addDisplacementMarker(() -> runTrajectory(rightShot))
                .build();

        rightShot = drive.trajectoryBuilder(missRings.end())
                .lineToSplineHeading(LauncherUtils.autoGetPowerPose(LauncherUtils.Position.RIGHT))
                .addDisplacementMarker(() -> {
                    drive.waitForFlywheel(LauncherUtils.flywheelThreshold);
                    drive.pressTrigger(true);
                    sleep(LauncherUtils.triggerActuationTime);
                    drive.pressTrigger(false);
                    drive.revFlywheel(-AutoPowerConstants.veloCenter);
                })
                .addDisplacementMarker(() -> runTrajectory(midShot))
                .build();

        nextTelemetry();
        midShot = drive.trajectoryBuilder(rightShot.end())
                .lineToSplineHeading(LauncherUtils.autoGetPowerPose(LauncherUtils.Position.CENTER))
                .addDisplacementMarker(() -> {
                    drive.waitForFlywheel(LauncherUtils.flywheelThreshold);
                    drive.pressTrigger(true);
                    sleep(LauncherUtils.triggerActuationTime);
                    drive.pressTrigger(false);
                    drive.revFlywheel(-AutoPowerConstants.veloLeft);
                })
                .addDisplacementMarker(() -> runTrajectory(leftShot))
                .build();

        nextTelemetry();
        leftShot = drive.trajectoryBuilder(midShot.end())
                .lineToSplineHeading(LauncherUtils.autoGetPowerPose(LauncherUtils.Position.LEFT))
                .addDisplacementMarker(() -> {
                    drive.waitForFlywheel(LauncherUtils.flywheelThreshold);
                    drive.pressTrigger(true);
                    sleep(LauncherUtils.triggerActuationTime);
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
                .lineToSplineHeading(AutoConstants.FirstBoxConstants.getBoxPose(AutoConstants.Box.A))
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
                .lineToSplineHeading(AutoConstants.FirstBoxConstants.getBoxPose(AutoConstants.Box.B))
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
                .lineToSplineHeading(AutoConstants.FirstBoxConstants.getBoxPose(AutoConstants.Box.C))
                .addDisplacementMarker(() -> {
                    try {
                        MoveWobble.depositWobble(drive);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    runTrajectory(toLineC);
                })
                .build();

        nextTelemetry();

        toLineC = drive.trajectoryBuilder(dropC.end())
                .lineToSplineHeading(new Pose2d(12, dropC.end().getY(),dropC.end().getHeading()))
                .addDisplacementMarker(() -> runningItem.setValue("done"))
                .build();

        nextTelemetry();

        telemetry.removeItem(trajBuildItem);
        initItem.setValue(String.format("Done. Took %f milliseconds",runtime.milliseconds()));
        telemetry.update();

        waitForStart();

        ringPosSaved = drive.getPosition();
        if(isStopRequested()) return;
        telemetry.removeItem(initItem);
        double initTime = runtime.milliseconds();

        drive.followTrajectoryAsync(missRings);

        Telemetry.Item runtimeItem = telemetry.addData(
                "Runtime",
                String.format(
                        "%fms",
                        runtime.milliseconds()-initTime
                ));
        runningItem.setValue("rightShot");
        telemetry.update();
        drive.revFlywheel(-AutoPowerConstants.veloRight);

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

            ringAnal.setValue(drive.getAnalysis());

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
