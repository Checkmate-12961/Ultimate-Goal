package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.HungryHippoDrive;

import java.util.List;
import java.util.Locale;

@TeleOp(group = "TeleOP")
@SuppressWarnings("unused")
@Config
public class TestingOP extends BaseOP{
    private final ElapsedTime runtime = new ElapsedTime();

    public static double moveX = 0;
    public static double moveY = 0;
    public static double rotate = 0;

    private static double leftEncoderOffset = 0;
    private static double rightEncoderOffset = 0;
    private static double frontEncoderOffset = 0;

    @Override
    public void runOpMode() throws InterruptedException{// Initialize SampleMecanumDrive
        HungryHippoDrive drive = new HungryHippoDrive(hardwareMap);

        // Reset the pose so we can virtually start at 0,0,0
        drive.setPoseEstimate(new Pose2d(0,0,0));

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            controlRobo(drive, runtime);
        }
    }

    @Override
    protected void controlRobo(HungryHippoDrive robot, ElapsedTime runtime) {
        controlRoboBase(robot, 0, false);
        Pose2d poseEstimate = robot.getPoseEstimate();
        // Print pose to telemetry
        telemetry.addData("POS",String.format(Locale.ENGLISH, "(%5f, %5f) @ %5f°",
                poseEstimate.getX(),
                poseEstimate.getY(),
                Math.toDegrees(poseEstimate.getHeading())
            ));
        telemetry.addData("FlyRPM", robot.getFlywheelVelo());
        telemetry.addData("Runtime",String.format(Locale.ENGLISH,"%fs",runtime.seconds()));
        List<Double> deadPos = robot.getDeadPositions();
        telemetry.addData("LeftEncoder",deadPos.get(0) - leftEncoderOffset);
        telemetry.addData("RightEncoder",deadPos.get(1) - rightEncoderOffset);
        telemetry.addData("FrontEncoder",deadPos.get(2) - frontEncoderOffset);
        telemetry.addData("RingAmount", robot.getPosition());
        telemetry.update();
    }

    @Override
    protected void controlRoboBase(HungryHippoDrive robot, double rotationalOffset, boolean relative) {
        // Update everything. Odometry. Etc.
        robot.update();

        switch (controlMode){
            case TELE:
                robot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                // Does things with coordinates from the dashboard
                goToPos(robot);
                goFromPos(robot);

                // Moves the robot based on the GP1 left stick
                runDrivetrain(robot, rotationalOffset, relative);


                // Resets and Zeros
                zeroPosition(robot);
                toZero(robot);
                break;

            case AUTO:
                robot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if (gamepad1.dpad_down) robot.cancelTrajectory();
                if (!robot.isBusy()) controlMode = ControlMode.TELE;
                break;

            default:
                break;
        }
    }

    // Goes to a position
    private void goToPos(HungryHippoDrive robot){
        if (gamepad1.a && (moveX != 0 || moveY != 0)){
            controlMode = ControlMode.AUTO;
            Pose2d here = robot.getPoseEstimate();
            Pose2d addPose = new Pose2d(moveX,moveY,Math.toRadians(rotate));
            Trajectory goTo = robot.trajectoryBuilder(here)
                    .lineToSplineHeading(here.plus(addPose))
                    .build();
            robot.followTrajectoryAsync(goTo);
        }
    }

    // Goes from a position
    private void goFromPos(HungryHippoDrive robot){
        if (gamepad1.b && (moveX != 0 || moveY != 0)){
            controlMode = ControlMode.AUTO;
            Pose2d here = robot.getPoseEstimate();
            Pose2d addPose = new Pose2d(moveX,moveY,Math.toRadians(rotate));
            Trajectory goTo = robot.trajectoryBuilder(here)
                    .lineToSplineHeading(here.minus(addPose))
                    .build();
            robot.followTrajectoryAsync(goTo);
        }
    }

    // Zeros the robot
    private void zeroPosition(HungryHippoDrive robot){
        if (gamepad1.dpad_up){
            robot.setPoseEstimate(new Pose2d(0,0,0));
            List<Double> deadPos = robot.getDeadPositions();
            leftEncoderOffset = deadPos.get(0);
            rightEncoderOffset = deadPos.get(1);
            frontEncoderOffset = deadPos.get(2);
        }
    }

    // Goes to what the robot thinks is zero
    private void toZero(HungryHippoDrive robot){
        Pose2d here = robot.getPoseEstimate();
        if (gamepad1.dpad_right && (here.getX() != 0 || here.getY() != 0)){
            controlMode = ControlMode.AUTO;
            Trajectory goTo = robot.trajectoryBuilder(here)
                    .lineToSplineHeading(new Pose2d(0,0,0))
                    .build();
            robot.followTrajectoryAsync(goTo);
        }
    }
}
