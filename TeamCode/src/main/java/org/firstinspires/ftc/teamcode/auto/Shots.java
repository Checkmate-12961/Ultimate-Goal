package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DrunkenHippoDrive;
import org.firstinspires.ftc.teamcode.drive.LauncherUtils;
import org.firstinspires.ftc.teamcode.drive.PoseUtils;
import org.firstinspires.ftc.teamcode.drive.launcherConstants.AutoPowerConstants;

import java.util.Locale;

@Disabled
@SuppressWarnings("unused")
@Autonomous(group = "Alcohol")
public class Shots extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();

    private Trajectory toLine;
    @SuppressWarnings("FieldCanBeLocal")
    private Trajectory rightShot;
    private Trajectory leftShot;
    private Trajectory midShot;

    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry.setAutoClear(false);
        Telemetry.Item initItem = telemetry.addData("Initializing...","Setting up hardware");
        telemetry.update();

        // RR stuff
        DrunkenHippoDrive drive = new DrunkenHippoDrive(hardwareMap);
        PoseUtils.currentPose = new Pose2d(-62, -19.5 , Math.toRadians(0) );
        Pose2d startPose = PoseUtils.currentPose;
        drive.setPoseEstimate(startPose);

        Telemetry.Item xItem = telemetry.addData("x",drive.getPoseEstimate().getX());
        Telemetry.Item yItem = telemetry.addData("y",drive.getPoseEstimate().getY());
        Telemetry.Item headingItem = telemetry.addData("θ",drive.getPoseEstimate().getHeading());

        initItem.setValue("Building trajectories");

        int onTrajBuild = 0;
        Telemetry.Item trajBuildItem = telemetry.addData("Built", onTrajBuild);
        telemetry.update();

        rightShot = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(LauncherUtils.autoGetPowerPose(LauncherUtils.Position.RIGHT))
                .addDisplacementMarker(() -> {
                    drive.waitForFlywheel(LauncherUtils.flywheelThreshold);
                    drive.pressTrigger(true);
                    sleep(LauncherUtils.triggerActuationTime);
                    drive.pressTrigger(false);
                    drive.revFlywheel(-AutoPowerConstants.veloCenter);
                })
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(midShot))
                .build();

        onTrajBuild = nextTelemetry(onTrajBuild,trajBuildItem);
        midShot = drive.trajectoryBuilder(rightShot.end())
                .lineToSplineHeading(LauncherUtils.autoGetPowerPose(LauncherUtils.Position.CENTER))
                .addDisplacementMarker(() -> {
                    drive.waitForFlywheel(LauncherUtils.flywheelThreshold);
                    drive.pressTrigger(true);
                    sleep(LauncherUtils.triggerActuationTime);
                    drive.pressTrigger(false);
                    drive.revFlywheel(-AutoPowerConstants.veloLeft);
                })
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(leftShot))
                .build();

        onTrajBuild = nextTelemetry(onTrajBuild,trajBuildItem);
        leftShot = drive.trajectoryBuilder(midShot.end())
                .lineToSplineHeading(LauncherUtils.autoGetPowerPose(LauncherUtils.Position.LEFT))
                .addDisplacementMarker(() -> {
                    drive.waitForFlywheel(LauncherUtils.flywheelThreshold);
                    drive.pressTrigger(true);
                    sleep(LauncherUtils.triggerActuationTime);
                    drive.pressTrigger(false);
                    drive.revFlywheel(0);
                })
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(toLine))
                .build();

        onTrajBuild = nextTelemetry(onTrajBuild,trajBuildItem);
        //toLine moves the robot straight forward to the line
        toLine = drive.trajectoryBuilder(leftShot.end())
                .splineTo(new Vector2d(12, leftShot.end().getY()), Math.toRadians(0))
                .build();
        nextTelemetry(onTrajBuild,trajBuildItem);

        telemetry.removeItem(trajBuildItem);
        Telemetry.Item ringPosEst = telemetry.addData("RingPosEst", drive.getPosition());
        Telemetry.Item ringAnal = telemetry.addData("RingAnalysis", drive.getAnalysis());
        initItem.setValue(String.format(Locale.ENGLISH, "Done. Took %f milliseconds",runtime.milliseconds()));
        telemetry.update();

        waitForStart();

        if(isStopRequested()) return;
        telemetry.removeItem(initItem);
        double initTime = runtime.milliseconds();

        drive.followTrajectoryAsync(rightShot);

        Telemetry.Item runtimeItem = telemetry.addData(
                "Runtime",
                String.format(Locale.ENGLISH, "%fms", runtime.milliseconds() - initTime));
        telemetry.update();

        drive.revFlywheel(-AutoPowerConstants.veloRight);

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            runtimeItem.setValue(
                    String.format(Locale.ENGLISH, "%fms", runtime.milliseconds() - initTime));
            Pose2d tempPose = drive.getPoseEstimate();
            xItem.setValue(tempPose.getX());
            yItem.setValue(tempPose.getY());

            ringPosEst.setValue(drive.getPosition());
            ringAnal.setValue(drive.getAnalysis());

            headingItem.setValue(tempPose.getHeading());
            telemetry.update();
        }
        PoseUtils.currentPose = drive.getPoseEstimate();
    }

    private int nextTelemetry(int onVal, Telemetry.Item telemetryItem){
        int newOnVal = onVal +1;
        telemetryItem.setValue(newOnVal);
        telemetry.update();
        return newOnVal;
    }
}
