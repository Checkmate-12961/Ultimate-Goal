package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class autoTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-70.5, -20.25, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        Trajectory traj0 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-30,-24),0)
                .splineTo(new Vector2d(45,-36), Math.toRadians(180))
                .build();
        Trajectory traj1 = drive.trajectoryBuilder(traj0.end())
                .lineToSplineHeading(new Pose2d(60, -36, Math.toRadians(180)))
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .splineTo(new Vector2d(10,-20),Math.toRadians(180))
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(traj0);
        drive.followTrajectory(traj1);
        drive.setIntakePowers(0, .5, .5);
        sleep(10000);
        drive.setIntakePowers(0,0,0);
        drive.followTrajectory(traj2);
    }
}