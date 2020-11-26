package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This opmode demonstrates how one would implement field centric control using
 * `SampleMecanumDrive.java`. This file is essentially just `TeleOpDrive.java` with the addition of
 * field centric control. To achieve field centric control, the only modification one needs is to
 * rotate the input vector by the current heading before passing it into the inverse kinematics.
 * <p>
 * See lines 42-57.
 */
@TeleOp(group = "TeleOP")
public class RelativeOP extends BaseOP {
    private final ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {
        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();

        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            controlRoboRelative(drive, 0,runtime);
        }
        PoseStorage.currentPose = drive.getPoseEstimate();
    }
    public void controlRoboRelative(SampleMecanumDrive robot, double rotationalOffset){
        controlRobo(robot, rotationalOffset, true);
    }
    public void controlRoboRelative(SampleMecanumDrive robot, double rotationalOffset, ElapsedTime runtime){
        controlRobo(robot, rotationalOffset, true, runtime);
    }
}