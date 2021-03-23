package org.firstinspires.ftc.teamcode.Teleop;

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
import org.firstinspires.ftc.teamcode.drive.HungryHippoDrive;

import java.util.Locale;

@TeleOp(group = "TeleOP")
public class BaseOP extends LinearOpMode {
    // Don't make this anything but private
    // We don't want this re-used between opmodes.
    private final ElapsedTime runtime = new ElapsedTime();

    @SuppressWarnings("FieldCanBeLocal")
    private Trajectory rightShot;
    private Trajectory midShot;
    private Trajectory leftShot;
    @SuppressWarnings("FieldCanBeLocal")
    private Trajectory shootPos;

    protected enum ControlMode {TELE, AUTO}
    protected ControlMode controlMode = ControlMode.TELE;

    @Override
    public void runOpMode() throws InterruptedException{
        // Initialize SampleMecanumDrive
        HungryHippoDrive drive = new HungryHippoDrive(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseUtils.currentPose);

        drive.setWobblePosPow(1,0);

        waitForStart();

        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            controlRobo(drive, 0,false, runtime);
        }
        PoseUtils.currentPose = PoseUtils.globalStartPose;
    }

    protected void controlRoboBase(HungryHippoDrive robot, double rotationalOffset, boolean relative) {
        // Update everything. Odometry. Etc.
        robot.update();

        switch (controlMode){
            case TELE:
                robot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                // Moves the robot based on the GP1 left stick
                runDrivetrain(robot, rotationalOffset, relative);

                // Control the intake motors
                runIntake(robot);

                // Control the wobble bits
                runWobble(robot);

                // Control the shooting mechanism
                runShooter(robot);

                // Positions robot and shoots into the power shots
                runSeekPowerShots(robot);

                // Zeroes the robot
                runZero(robot);

                // Moves the robot to hit the high goal
                runSeekHighGoal(robot);
                break;

            case AUTO:
                // Replace false here with a check to cancel the trajectory
                robot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //noinspection ConstantConditions
                if (false) robot.cancelTrajectory();
                if (!robot.isBusy()) controlMode = ControlMode.TELE;
                break;

            default:
                break;
        }
    }

    protected void controlRobo(HungryHippoDrive robot, double rotationalOffset, boolean relative, ElapsedTime runtime) {
        controlRoboBase(robot, rotationalOffset, relative);
        Pose2d poseEstimate = robot.getPoseEstimate();
        // Print pose to telemetry
        telemetry.addData("x", PoseUtils.currentPose.getX());
        telemetry.addData("y", PoseUtils.currentPose.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.addData("flyRate", robot.getFlywheelVelo());
        telemetry.addData("runtime",String.format(Locale.ENGLISH,"%fs",runtime.seconds()));
        telemetry.update();
    }
    protected void runDrivetrain (HungryHippoDrive robot, double rotationalOffset, boolean relative){
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
    }
    private void runIntake (HungryHippoDrive robot){
        double intakePower = 0;
        double transferPower = 0;
        if (gamepad2.a) transferPower += 1;
        if (gamepad2.b) transferPower -= 1;
        if (gamepad2.y) intakePower += 1;
        if (gamepad2.x) intakePower -= 1;
        robot.setIntakePowers(-intakePower, transferPower);
    }
    private void runWobble (HungryHippoDrive robot){
        int grab = 0;
        if (gamepad2.dpad_right) grab += 1;
        if (gamepad2.dpad_left) grab -= 1;

        robot.setWobblePosPow(grab, gamepad2.right_stick_y/2);
    }
    private void runShooter (HungryHippoDrive robot){
        // rev flywheel
        if (gamepad2.left_trigger > .9) robot.revFlywheel(-LauncherConstants.highGoalVelo);
        else if (gamepad2.left_bumper) robot.revFlywheel(0);

        // Shoots circle at target.
        robot.pressTrigger(gamepad1.left_trigger > .9);
    }
    private void runSeekPowerShots (HungryHippoDrive robot){
        if (gamepad1.dpad_right) {
            controlMode = ControlMode.AUTO;
            //I'm so sorry for this it's chaos, but it works sort of.
            //Changes robot position estimate to the side of the field, so roadrunner is more consistent
            robot.setPoseEstimate(new Pose2d(2.5, -61.25, Math.toRadians(0)));
            //Revs flywheel in advance.
            robot.revFlywheel(-LauncherConstants.powerShotVeloRight);
            //One trajectory defined for each of the high goals.
            rightShot = robot.trajectoryBuilder(new Pose2d(2.5, -61.25, 0))
                    //Move to shooting locations
                    .lineToSplineHeading(LauncherConstants.getPowerPose(Math.toRadians(LauncherConstants.powerShotAngle)))
                    .addDisplacementMarker(() -> {
                        //Wait for the flywheel to get to full speed. This line is unique to this block.
                        robot.waitForFlywheel(LauncherConstants.flywheelThreshold);
                        //Virtually presses trigger
                        robot.pressTrigger(true);
                        //Gives the trigger time to finish shooting.
                        sleep(LauncherConstants.triggerActuationTime);
                        //Retracts trigger
                        robot.pressTrigger(false);
                        robot.revFlywheel(-LauncherConstants.powerShotVeloCenter);
                        robot.followTrajectoryAsync(midShot);
                    })
                    .build();
            midShot = robot.trajectoryBuilder(rightShot.end())
                    .lineToSplineHeading(new Pose2d(LauncherConstants.powerShotX, LauncherConstants.powerShotY + LauncherConstants.pegDist, Math.toRadians(LauncherConstants.powerShotAngle + LauncherConstants.rotFix)))
                    .addDisplacementMarker(() -> {
                        robot.waitForFlywheel(LauncherConstants.flywheelThreshold);
                        robot.pressTrigger(true);
                        sleep(LauncherConstants.triggerActuationTime);
                        robot.pressTrigger(false);
                        robot.revFlywheel(-LauncherConstants.powerShotVeloLeft);
                        robot.followTrajectoryAsync(leftShot);
                    })
                    .build();
            leftShot = robot.trajectoryBuilder(midShot.end())
                    .lineToSplineHeading(new Pose2d(LauncherConstants.powerShotX, LauncherConstants.powerShotY + LauncherConstants.pegDist * 2, Math.toRadians(LauncherConstants.powerShotAngle + LauncherConstants.rotFix * 2)))
                    .addDisplacementMarker(() -> {
                        robot.waitForFlywheel(LauncherConstants.flywheelThreshold);
                        robot.pressTrigger(true);
                        sleep(LauncherConstants.triggerActuationTime);
                        robot.pressTrigger(false);

                        //Turns the flywheel off
                        robot.revFlywheel(0);
                    })
                    .build();

            //These three blocks are almost identical.
            robot.followTrajectoryAsync(rightShot);
        }
    }
    private void runZero (HungryHippoDrive robot) {
        if (gamepad1.dpad_left) {
            robot.setPoseEstimate(new Pose2d(2.5, -61.25, Math.toRadians(0)));
        }
    }
    private void runSeekHighGoal (HungryHippoDrive robot){
        if (gamepad1.dpad_up) {
            controlMode = ControlMode.AUTO;
            shootPos = robot.trajectoryBuilder(robot.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(LauncherConstants.highGoalX, LauncherConstants.highGoalY, Math.toRadians(LauncherConstants.highGoalAngle)))
                    .build();
            robot.followTrajectoryAsync(shootPos);
        }
    }
}