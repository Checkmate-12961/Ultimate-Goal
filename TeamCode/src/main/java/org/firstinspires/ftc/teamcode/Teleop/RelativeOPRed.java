package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.PoseUtils;
import org.firstinspires.ftc.teamcode.drive.HungryHippoDrive;

@Deprecated
@Disabled
@TeleOp(name="Tele Relative (R)", group="TeleOP")
public class RelativeOPRed extends RelativeOP {
    private final ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        HungryHippoDrive drive = new HungryHippoDrive(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseUtils.currentPose);

        waitForStart();

        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            controlRoboRelative(drive, -Math.PI / 2,runtime);
        }
        PoseUtils.currentPose = drive.getPoseEstimate();
    }
}
