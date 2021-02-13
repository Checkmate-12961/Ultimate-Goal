package org.firstinspires.ftc.teamcode.Teleop;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.LauncherMath;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group = "TeleOP")
public class BaseOP extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException{
        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // TODO: REMOVE THIS AND WRITE A PROPER ZERO FUNCTION!!!
        PoseStorage.currentPose = new Pose2d(-70.5, -20.25 , Math.toRadians(0) );
        // DON'T BE LAZY, RILEY!!!


        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();

        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            controlRobo(drive, 0,false, runtime);
        }
        PoseStorage.currentPose = drive.getPoseEstimate();
    }
    private void controlRoboBase(SampleMecanumDrive robot, double rotationalOffset, boolean relative) {
        // Read pose
        Pose2d poseEstimate = robot.getPoseEstimate();
        double offset;
        if (relative) offset = -rotationalOffset - poseEstimate.getHeading();
        else offset = 0;

        double xin = gamepad1.left_stick_x * Range.scale((gamepad1.right_trigger - gamepad1.left_trigger), -1, 1, 0, 1);
        double yin = gamepad1.left_stick_y * Range.scale((gamepad1.right_trigger - gamepad1.left_trigger), -1, 1, 0, 1);

        // Create a vector from the gamepad x/y inputs
        // Then, rotate that vector by the inverse of that heading
        Vector2d input = new Vector2d(
                -yin,
                -xin
        ).rotated(offset);  

        // Pass in the rotated input + right stick value for rotation
        // Rotation is not part of the rotated input thus must be passed in separately
        robot.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -gamepad1.right_stick_x * Range.scale((gamepad1.right_trigger - gamepad1.left_trigger), -1, 1, 0, 1)
                )
        );

        // Control the intake motors
        double intakePower = 0;
        double transferPower = 0;
        if (gamepad2.a) transferPower += 1;
        if (gamepad2.b) transferPower -= 1;
        if (gamepad2.y) intakePower += 1;
        if (gamepad2.x) intakePower -= 1;
        robot.setIntakePowers(-intakePower, transferPower);

        // Control the wobble bits
        int grab = 0;
        if (gamepad2.right_bumper) grab += 1;
        if (gamepad2.left_bumper) grab -= 1;

        int arm = 0;
        if (gamepad2.dpad_up) arm   += 1;
        if (gamepad2.dpad_down) arm -= 1;

        robot.setWobblePosPow(grab, arm);

        // rev flywheel
        if (gamepad2.left_trigger > .9) robot.revFlywheel(-LauncherMath.power);
        else if (gamepad2.left_bumper) robot.revFlywheel(0);


        // Shoots circle at target.
        // _TODO: adjust the trigger sensitivity by changing the decimal number
        robot.pressTrigger(gamepad2.right_trigger > .9);

        // Positions robot to shoot into the Goals
        if (gamepad1.dpad_right) {
            robot.setPoseEstimate(new Pose2d(-73, -63, Math.toRadians(0)));
            robot.revFlywheel(-LauncherMath.powerShotPower);

            Trajectory rightShot = robot.trajectoryBuilder(new Pose2d(-73, -63, Math.toRadians(LauncherMath.angle)))
                    .splineTo(new Vector2d(LauncherMath.rightX, LauncherMath.rightY), Math.toRadians(LauncherMath.angle+15))
                    .build();
            Trajectory midShot = robot.trajectoryBuilder(rightShot.end())
                    .lineToSplineHeading(new Pose2d(LauncherMath.rightX, LauncherMath.rightY+8, Math.toRadians(LauncherMath.angle)))
                    .build();
            Trajectory leftShot = robot.trajectoryBuilder(midShot.end())
                    .lineToSplineHeading(new Pose2d(LauncherMath.rightX, LauncherMath.rightY+15, Math.toRadians(LauncherMath.angle)))
                    .build();

            robot.followTrajectory(rightShot);
            sleep(2000);
            robot.pressTrigger(true);
            sleep(LauncherMath.shootLength);
            robot.pressTrigger(false);

            robot.followTrajectory(midShot);
            sleep(LauncherMath.shootCoolDown);
            robot.pressTrigger(true);
            sleep(LauncherMath.shootLength);
            robot.pressTrigger(false);
            sleep(LauncherMath.shootCoolDown);

            robot.followTrajectory(leftShot);
            sleep(LauncherMath.shootCoolDown);
            robot.pressTrigger(true);
            sleep(LauncherMath.shootLength);
            robot.pressTrigger(false);
            sleep(LauncherMath.shootCoolDown);

            robot.revFlywheel(0);
        }


        if (gamepad1.dpad_up) {
            Trajectory shootPos = robot.trajectoryBuilder(robot.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(LauncherMath.toX, LauncherMath.toY, Math.toRadians(LauncherMath.angle)))
                    .build();
            robot.followTrajectory(shootPos);
        }

        // Update everything. Odometry. Etc.
        robot.update();
    }
    @SuppressLint("DefaultLocale")
    public void controlRobo(SampleMecanumDrive robot, double rotationalOffset, boolean relative, ElapsedTime runtime) {
        controlRoboBase(robot, rotationalOffset, relative);
        Pose2d poseEstimate = robot.getPoseEstimate();
        // Print pose to telemetry
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.addData("runtime",String.format("%fs",runtime.seconds()));
        telemetry.update();
    }
    public void controlRobo(SampleMecanumDrive robot, double rotationalOffset, boolean relative) {
        controlRoboBase(robot, rotationalOffset, relative);
        Pose2d poseEstimate = robot.getPoseEstimate();
        // Print pose to telemetry
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.update();
    }
}