package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.LauncherUtils;
import org.firstinspires.ftc.teamcode.drive.PoseUtils;
import org.firstinspires.ftc.teamcode.drive.DrunkenHippoDrive;
import org.firstinspires.ftc.teamcode.drive.launcherConstants.EndgamePowerConstants;
import org.firstinspires.ftc.teamcode.drive.launcherConstants.HighGoalConstants;

import java.util.Locale;

@Config
@TeleOp(group = "TeleOP")
public class BaseOP extends LinearOpMode {
    // Don't make this anything but private
    // We don't want this re-used between opmodes.
    private final ElapsedTime runtime = new ElapsedTime();

    // These numbers tell the robot where it is when the
    //  endgame powershots get triggered.
    public static double setPointX = 2.5;
    public static double setPointY = -61.25;
    public static double setPointHeading = -2.6;
    
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
        DrunkenHippoDrive drive = new DrunkenHippoDrive(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseUtils.currentPose);

        waitForStart();

        drive.dropStop(DrunkenHippoDrive.RingStopPos.START);

        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            controlRobo(drive, runtime);
        }
        PoseUtils.currentPose = PoseUtils.getStartPose();
    }

    protected void controlRoboBase(DrunkenHippoDrive robot, double rotationalOffset, boolean relative) {
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

                // Moved the ring stopper to the correct position
                runDropStop(robot);
                break;

            case AUTO:
                robot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                // Replace false here with a check to cancel the trajectory
                //noinspection ConstantConditions
                if (false) robot.cancelTrajectory();
                if (!robot.isBusy()) controlMode = ControlMode.TELE;
                break;
            default:
                // If we end up here, something went horribly wrong.
                // Generally, the best plan of action is to ignore
                //  it and move on.
                controlMode = ControlMode.TELE;
                // Mission accomplished.
                break;
        }
    }

    protected void controlRobo(DrunkenHippoDrive robot, ElapsedTime runtime) {
        controlRoboBase(robot, 0, false);
        PoseUtils.currentPose = robot.getPoseEstimate();
        // Print pose to telemetry
        telemetry.addData("x", PoseUtils.currentPose.getX());
        telemetry.addData("y", PoseUtils.currentPose.getY());
        telemetry.addData("heading", Math.toDegrees(PoseUtils.currentPose.getHeading()));
        telemetry.addData("flyRate", robot.getFlywheelVelo());
        telemetry.addData("runtime",String.format(Locale.ENGLISH,"%fs",runtime.seconds()));
        telemetry.update();
    }

    @SuppressWarnings("unused")
    protected void controlRoboRelative(DrunkenHippoDrive robot, double rotationalOffset, ElapsedTime runtime) {
        controlRoboBase(robot, rotationalOffset, true);
        PoseUtils.currentPose = robot.getPoseEstimate();
        // Print pose to telemetry
        telemetry.addData("x", PoseUtils.currentPose.getX());
        telemetry.addData("y", PoseUtils.currentPose.getY());
        telemetry.addData("heading", Math.toDegrees(PoseUtils.currentPose.getHeading()));
        telemetry.addData("flyRate", robot.getFlywheelVelo());
        telemetry.addData("runtime",String.format(Locale.ENGLISH,"%fs",runtime.seconds()));
        telemetry.update();
    }

    // BIND:
    //  gamepad1.left_stick_x, gamepad1.left_stick_y
    //  gamepad1.right_stick_x, gamepad1.right_stick_y
    //  gamepad1.right_trigger
    protected void runDrivetrain (DrunkenHippoDrive robot, double rotationalOffset, boolean relative){
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

    // BIND: gamepad2.a, gamepad2.b, gamepad2.x, gamepad2.y
    private void runIntake (DrunkenHippoDrive robot){
        double intakePower = 0;
        if (gamepad2.a) intakePower += 1;
        if (gamepad2.b) intakePower -= 1;
        robot.setIntakePowers(intakePower);
    }

    // BIND: gamepad2.dpad_right, gamepad2.dpad_left, gamepad2.right_stick_y
    private void runWobble (DrunkenHippoDrive robot){
        if (gamepad2.dpad_left){
            if (!gamepad2.dpad_right){
                robot.setWobbleGrab(DrunkenHippoDrive.WobbleGrabPos.END);
            }
        } else if (gamepad2.dpad_right){
            robot.setWobbleGrab(DrunkenHippoDrive.WobbleGrabPos.START);
        }

        robot.setWobblePivot(gamepad2.right_stick_y/2);
    }

    // BIND: gamepad2.left_trigger, gamepad2.left_bumper, gamepad1.left_trigger
    private void runShooter (DrunkenHippoDrive robot){
        // rev flywheel
        if (gamepad2.left_trigger > .9) robot.revFlywheel(-HighGoalConstants.velo);
        else if (gamepad2.left_bumper) robot.revFlywheel(0);

        // Shoots circle at target.
        robot.pressTrigger(gamepad1.left_trigger > .9);
    }

    // BIND: gamepad1.dpad_right
    private void runSeekPowerShots (DrunkenHippoDrive robot){
        if (gamepad1.dpad_right) {
            controlMode = ControlMode.AUTO;
            while (setPointHeading > 360){
                setPointHeading -= 360;
            } while (setPointHeading < 0){
                setPointHeading += 360;
            }
            //Changes robot position estimate to the side of the field, so roadrunner is more consistent
            robot.setPoseEstimate(new Pose2d(setPointX, setPointY, Math.toRadians(setPointHeading)));
            //Revs flywheel in advance.
            robot.revFlywheel(-EndgamePowerConstants.veloRight);
            //One trajectory defined for each of the high goals.
            rightShot = robot.trajectoryBuilder(new Pose2d(setPointX, setPointY, Math.toRadians(setPointHeading)))
                    //Move to shooting locations
                    .lineToSplineHeading(LauncherUtils.getPowerPose(LauncherUtils.Position.RIGHT))
                    .addDisplacementMarker(() -> {
                        //Wait for the flywheel to get to full speed. This line is unique to this block.
                        robot.waitForFlywheel(LauncherUtils.flywheelThreshold);
                        //Virtually presses trigger
                        robot.pressTrigger(true);
                        //Gives the trigger time to finish shooting.
                        sleep(LauncherUtils.triggerActuationTime);
                        //Retracts trigger
                        robot.pressTrigger(false);
                        robot.revFlywheel(-EndgamePowerConstants.veloCenter);
                        robot.followTrajectoryAsync(midShot);
                    })
                    .build();
            midShot = robot.trajectoryBuilder(rightShot.end())
                    .lineToSplineHeading(LauncherUtils.getPowerPose(LauncherUtils.Position.CENTER))
                    .addDisplacementMarker(() -> {
                        robot.waitForFlywheel(LauncherUtils.flywheelThreshold);
                        robot.pressTrigger(true);
                        sleep(LauncherUtils.triggerActuationTime);
                        robot.pressTrigger(false);
                        robot.revFlywheel(-EndgamePowerConstants.veloLeft);
                        robot.followTrajectoryAsync(leftShot);
                    })
                    .build();
            leftShot = robot.trajectoryBuilder(midShot.end())
                    .lineToSplineHeading(LauncherUtils.getPowerPose(LauncherUtils.Position.LEFT))
                    .addDisplacementMarker(() -> {
                        robot.waitForFlywheel(LauncherUtils.flywheelThreshold);
                        robot.pressTrigger(true);
                        sleep(LauncherUtils.triggerActuationTime);
                        robot.pressTrigger(false);

                        //Turns the flywheel off
                        robot.revFlywheel(0);
                    })
                    .build();

            //These three blocks are almost identical.
            robot.followTrajectoryAsync(rightShot);
        }
    }

    // BIND: gamepad1.dpad_left
    private void runZero (DrunkenHippoDrive robot) {
        if (gamepad1.dpad_left) {
            robot.setPoseEstimate(new Pose2d(setPointX, setPointY, Math.toRadians(setPointHeading)));
        }
    }

    // BIND: gamepad1.dpad_up
    private void runSeekHighGoal (DrunkenHippoDrive robot){
        if (gamepad1.dpad_up) {
            controlMode = ControlMode.AUTO;
            shootPos = robot.trajectoryBuilder(robot.getPoseEstimate())
                    .lineToSplineHeading(LauncherUtils.getHighGoalPose())
                    .build();
            robot.followTrajectoryAsync(shootPos);
        }
    }

    // BIND: gamepad1.left_bumper, gamepad1.right_bumper
    private void runDropStop (DrunkenHippoDrive robot){
        if (gamepad1.left_bumper){
            if (!gamepad1.right_bumper){
                robot.dropStop(DrunkenHippoDrive.RingStopPos.END);
            }
        } else if (gamepad1.right_bumper){
            robot.dropStop(DrunkenHippoDrive.RingStopPos.START);
        }
    }
}