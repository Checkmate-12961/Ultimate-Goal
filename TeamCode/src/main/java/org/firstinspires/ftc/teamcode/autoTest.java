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

        // traj0 and traj1 navigate the robot from the starting position to the goal
        Trajectory traj0 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-30,-24),0)
                .splineTo(new Vector2d(45,-36), Math.toRadians(180))
                .build();
        Trajectory traj1 = drive.trajectoryBuilder(traj0.end())
                .lineToSplineHeading(new Pose2d(64, -38, Math.toRadians(180)))
                .build();
        //traj2 navigates the robot to the middle line
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .splineTo(new Vector2d(10,-20),Math.toRadians(180))
                .build();

        waitForStart();

        if(isStopRequested()) return;

        //Executes traj0 and traj1, positioning the bot before the goal
        drive.followTrajectory(traj0);
        drive.followTrajectory(traj1);
        //dumps rings into low goal
        drive.setIntakePowers(0, .5, .5);
        sleep(6000);
        drive.setIntakePowers(0,0,0);
        //executes traj2, positioning the bot atop the middle line.
        drive.followTrajectory(traj2);
    }
}