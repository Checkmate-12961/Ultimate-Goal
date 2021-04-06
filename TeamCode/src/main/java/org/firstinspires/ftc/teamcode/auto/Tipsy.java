package org.firstinspires.ftc.teamcode.auto;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DrunkenHippoDrive;
import org.firstinspires.ftc.teamcode.drive.LauncherConstants;
import org.firstinspires.ftc.teamcode.drive.PoseUtils;

@Disabled
@Config
@Autonomous(group = "Alcohol")
@SuppressWarnings("unused")
public class Tipsy extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private DrunkenHippoDrive.RingPosition ringPosSaved;

    private Trajectory dropA;
    private Trajectory dropB;
    private Trajectory dropC;
    private Trajectory toLineToo;

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

        Trajectory toLine = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(LauncherConstants.autoGetPowerPose(LauncherConstants.Position.LEFT))
                .addDisplacementMarker(() -> {
                    if (ringPosSaved == DrunkenHippoDrive.RingPosition.NONE) {
                        drive.followTrajectoryAsync(dropA);
                    } else if (ringPosSaved == DrunkenHippoDrive.RingPosition.ONE) {
                        drive.followTrajectoryAsync(dropB);
                    } else if (ringPosSaved == DrunkenHippoDrive.RingPosition.FOUR) {
                        drive.followTrajectoryAsync(dropC);
                    }
                })
                .build();

        onTrajBuild = nextTelemetry(onTrajBuild,trajBuildItem);

        dropA = drive.trajectoryBuilder(toLine.end())
                .lineToSplineHeading(new Pose2d(dropAX,dropAY,dropAH))
                .addDisplacementMarker(() -> {
                    try {
                        MoveWobble.depositWobble(drive);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                })
                .build();

        onTrajBuild = nextTelemetry(onTrajBuild,trajBuildItem);

        dropB = drive.trajectoryBuilder(toLine.end())
                .lineToSplineHeading(new Pose2d(dropBX,dropBY,dropBH))
                .addDisplacementMarker(() -> {
                    try {
                        MoveWobble.depositWobble(drive);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                })
                .build();

        onTrajBuild = nextTelemetry(onTrajBuild,trajBuildItem);

        dropC = drive.trajectoryBuilder(toLine.end())
                .lineToSplineHeading(new Pose2d(dropCX,dropCY,dropCH))
                .addDisplacementMarker(() -> {
                    try {
                        MoveWobble.depositWobble(drive);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    drive.followTrajectoryAsync(toLineToo);
                })
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

        ringPosSaved = drive.getPosition();
        Telemetry.Item ringPosEst = telemetry.addData("RingPosEst", ringPosSaved);
        Telemetry.Item ringAnal = telemetry.addData("RingAnal", drive.getAnalysis());
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
            ringAnal.setValue(drive.getAnalysis());

            headingItem.setValue(tempPose.getHeading());
            telemetry.update();
        }
        PoseUtils.currentPose = drive.getPoseEstimate();
    }

    private int nextTelemetry(int onVal, Telemetry.Item telemetryItem){
        int nextVal = onVal + 1;
        telemetryItem.setValue(nextVal);
        telemetry.update();
        return nextVal;
    }

}
