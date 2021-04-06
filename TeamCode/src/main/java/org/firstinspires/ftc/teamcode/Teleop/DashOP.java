package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DrunkenHippoDrive;
import org.firstinspires.ftc.teamcode.drive.PoseUtils;

@Disabled
@TeleOp(group = "TeleOP")
@SuppressWarnings("unused")
@Config
public class DashOP  extends  BaseOP{
    private final ElapsedTime runtime = new ElapsedTime();

    public static double moveX = 0;
    public static double moveY = 0;
    public static double rotate = 0;
    public static double run = 0;

    @Override
    public void runOpMode() throws InterruptedException{
        // Initialize SampleMecanumDrive
        DrunkenHippoDrive drive = new DrunkenHippoDrive(hardwareMap);

        moveX = 0;
        moveY = 0;
        rotate = 0;
        run = 0;

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseUtils.currentPose);

        waitForStart();

        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            if (run > 0 && controlMode != ControlMode.AUTO){
                controlMode = ControlMode.AUTO;
                Pose2d here = drive.getPoseEstimate();
                Pose2d move = new Pose2d(moveX,moveY,Math.toRadians(rotate));
                Trajectory dashControl = drive.trajectoryBuilder(here)
                        .lineToSplineHeading(here.plus(move))
                        .addDisplacementMarker(() -> run = 0)
                        .build();
                drive.followTrajectoryAsync(dashControl);
            }
            controlRobo(drive, runtime);
        }
        PoseUtils.currentPose = PoseUtils.getStartPose();
    }
}
