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

import org.firstinspires.ftc.teamcode.drive.LauncherConstants;
import org.firstinspires.ftc.teamcode.drive.PoseUtils;
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

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseUtils.currentPose);

        drive.setWobblePosPow(-1,0);

        waitForStart();

        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            controlRobo(drive, 0,false, runtime);
        }
        PoseUtils.currentPose = PoseUtils.globalStartPose;
    }
    private void controlRoboBase(SampleMecanumDrive robot, double rotationalOffset, boolean relative) {
        // Read pose
        Pose2d poseEstimate = robot.getPoseEstimate();
        double offset;
        if (relative) offset = -rotationalOffset - poseEstimate.getHeading();
        else offset = 0;

        double xin = gamepad1.left_stick_x * Range.scale((gamepad1.right_trigger), -1, 1, 0, 1);
        double yin = gamepad1.left_stick_y * Range.scale((gamepad1.right_trigger), -1, 1, 0, 1);

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
                        -gamepad1.right_stick_x * Range.scale((gamepad1.right_trigger), -1, 1, 0, 1)
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
        if (gamepad2.dpad_right) grab += 1;
        if (gamepad2.dpad_left) grab -= 1;

        robot.setWobblePosPow(grab, gamepad2.right_stick_y/3);

        // rev flywheel
        if (gamepad2.left_trigger > .9) robot.revFlywheel(-LauncherConstants.highGoalVelo);
        else if (gamepad2.left_bumper) robot.revFlywheel(0);


        // Shoots circle at target.
        // _TODO: adjust the trigger sensitivity by changing the decimal number
        robot.pressTrigger(gamepad1.left_trigger > .9);

        // Positions robot to shoot into the power shots
        //I'm so sorry for this it's chaos, but it works sort of.
        if (gamepad1.dpad_right) {
            //Changes robot position estimate to a corner of the field, so roadrunner is more consistent
            robot.setPoseEstimate(new Pose2d(2.5, -61.25, Math.toRadians(0)));
            //Revs flywheel in advance.
            robot.revFlywheel(-LauncherConstants.powerShotVeloRight);
            //One trajectory defined for each of the high goals.
            Trajectory rightShot = robot.trajectoryBuilder(new Pose2d(2.5, -61.25, 0))
                    //.lineToSplineHeading(new Pose2d(-55,-55,0))
                    //.splineTo(new Vector2d(LauncherMath.rightX-10, LauncherMath.rightY-10), 0)
                    .lineToSplineHeading(LauncherConstants.getPowerPose(Math.toRadians(LauncherConstants.powerShotAngle)))
                    .build();
            Trajectory midShot = robot.trajectoryBuilder(rightShot.end())
                    .lineToSplineHeading(new Pose2d(LauncherConstants.powerShotX, LauncherConstants.powerShotY + LauncherConstants.pegDist, Math.toRadians(LauncherConstants.powerShotAngle+ LauncherConstants.rotFix)))
                    .build();
            Trajectory leftShot = robot.trajectoryBuilder(midShot.end())
                    .lineToSplineHeading(new Pose2d(LauncherConstants.powerShotX, LauncherConstants.powerShotY + LauncherConstants.pegDist *2, Math.toRadians(LauncherConstants.powerShotAngle+ LauncherConstants.rotFix*2)))
                    .build();

            //These three blocks are almost identical.
            //Move to shooting locations
            robot.followTrajectory(rightShot);
            //Wait for the flywheel to get to full speed. This line is unique to this block.
            sleep(1000);
            //Virtually presses trigger
            robot.pressTrigger(true);
            //Gives the trigger time to finish shooting.
            sleep(LauncherConstants.triggerActuationTime);
            //Retracts trigger
            robot.pressTrigger(false);
            robot.revFlywheel(-LauncherConstants.powerShotVeloCenter);

            robot.followTrajectory(midShot);
            sleep(LauncherConstants.shootCoolDown);
            robot.pressTrigger(true);
            sleep(LauncherConstants.triggerActuationTime);
            robot.pressTrigger(false);

            robot.revFlywheel(-LauncherConstants.powerShotVeloLeft);
            robot.followTrajectory(leftShot);
            sleep(LauncherConstants.shootCoolDown);
            robot.pressTrigger(true);
            sleep(LauncherConstants.triggerActuationTime);
            robot.pressTrigger(false);

            //Turns the flywheel off
            robot.revFlywheel(0);
        }

        if (gamepad1.dpad_left) {
            robot.setPoseEstimate(new Pose2d(2.5, -61.25, Math.toRadians(0)));
        }
        if (gamepad1.dpad_up) {
            Trajectory shootPos = robot.trajectoryBuilder(robot.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(LauncherConstants.highGoalX, LauncherConstants.highGoalY, Math.toRadians(LauncherConstants.highGoalAngle)))
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
        telemetry.addData("x", PoseUtils.currentPose.getX());
        telemetry.addData("y", PoseUtils.currentPose.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.addData("flyRate", robot.getFlywheelVelo());
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
        telemetry.addData("flyRate", robot.getFlywheelVelo());
        telemetry.update();
    }
}