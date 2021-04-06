package org.firstinspires.ftc.teamcode.auto;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.HungryHippoDrive;
import org.firstinspires.ftc.teamcode.drive.LauncherConstants;
import org.firstinspires.ftc.teamcode.drive.PoseUtils;

@SuppressWarnings("unused")
@Autonomous(group = "Alcohol")
public class Whiteclaw extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private HungryHippoDrive.RingPosition ringPosSaved;

    private enum RunMode {FIRST, RUNNING, SECOND, DONE}
    private RunMode currentMode = RunMode.FIRST;

    @SuppressWarnings("FieldCanBeLocal")
    private Trajectory missRings;
    private Trajectory rightShot;
    private Trajectory midShot;
    private Trajectory leftShot;
    private Trajectory dropA;
    private Trajectory dropB;
    private Trajectory dropC;
    private Trajectory toLineA;
    private Trajectory toLineB;
    private Trajectory toLineC;
    private Trajectory retrieveWobble;

    @SuppressWarnings("FieldCanBeLocal")
    private Trajectory grabWobble;
    private Trajectory missRings2;
    private Trajectory toLine;


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

        trajBuildItem = telemetry.addData("Built", onTrajBuild);
        telemetry.update();

        missRings = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(0, startPose.getY(), 0))
                .addDisplacementMarker(() -> runTrajectory(rightShot))
                .build();

        rightShot = drive.trajectoryBuilder(missRings.end())
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
                .lineToSplineHeading(AutoConstants.getBoxPose(AutoConstants.Box.A))
                .addDisplacementMarker(() -> {
                    try {
                        MoveWobble.depositWobble(drive);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    runTrajectory(toLineA);
                })
                .build();
        nextTelemetry();

        dropB = drive.trajectoryBuilder(leftShot.end())
                .lineToSplineHeading(AutoConstants.getBoxPose(AutoConstants.Box.B))
                .addDisplacementMarker(() -> {
                    try {
                        MoveWobble.depositWobble(drive);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    runTrajectory(toLineB);
                })
                .build();
        nextTelemetry();

        dropC = drive.trajectoryBuilder(leftShot.end())
                .lineToSplineHeading(AutoConstants.getBoxPose(AutoConstants.Box.C))
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

        toLineA = drive.trajectoryBuilder(dropA.end())
                //.lineToSplineHeading(new Vector2d(-12, -48), -Math.PI/2)
                .lineToSplineHeading(new Pose2d(-24, -54, 0))
                .addDisplacementMarker(() -> currentMode = RunMode.SECOND)
                .build();
        nextTelemetry();

        toLineB = drive.trajectoryBuilder(dropB.end())
                //.lineToSplineHeading(new Vector2d(-12, -36), -Math.PI/2)
                //.lineToSplineHeading(new Vector2d(-12, -48), -Math.PI/2)
                .lineToSplineHeading(new Pose2d(-24, -54, Math.toRadians(0)))
                .addDisplacementMarker(() -> currentMode = RunMode.SECOND)
                .build();
        nextTelemetry();

        toLineC = drive.trajectoryBuilder(dropC.end())
                //.lineToSplineHeading(new Vector2d(36,-36), Math.PI)
                //.lineToSplineHeading(new Vector2d(0, -48), Math.PI)
                .lineToSplineHeading(new Pose2d(-24, -54, Math.toRadians(0)))
                .addDisplacementMarker(() -> currentMode = RunMode.SECOND)
                .build();
        nextTelemetry();

        grabWobble = drive.trajectoryBuilder(new Pose2d(-24, -54,0))
                .lineToSplineHeading(new Pose2d(-37, -38.5, 0))
                .addDisplacementMarker(() -> {
                    try {
                        MoveWobble.collectWobble(drive);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    runTrajectory(missRings2);
                })
                .build();
        nextTelemetry();

        missRings2 = drive.trajectoryBuilder(grabWobble.end())
                .lineToSplineHeading(new Pose2d(-12, -54, 0))
                .addDisplacementMarker(() -> runTrajectory(toLine))
                .build();
        nextTelemetry();

        toLine = drive.trajectoryBuilder(missRings2.end())
                .lineToSplineHeading(new Pose2d(12,-36,0))
                .addDisplacementMarker(() -> currentMode = RunMode.DONE)
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
            switch (currentMode){
                case FIRST:
                    runTrajectory(missRings);
                    currentMode = RunMode.RUNNING;
                    break;
                case SECOND:
                    runTrajectory(grabWobble);
                    currentMode = RunMode.RUNNING;
                    break;
                default:
                    break;

            }
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
