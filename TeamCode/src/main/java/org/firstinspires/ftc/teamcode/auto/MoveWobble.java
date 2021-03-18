package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class MoveWobble extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        depositWobble(drive);

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
        }
    }
    public static void depositWobble(SampleMecanumDrive drive) throws InterruptedException {
        drive.setWobblePosPow(1,0);
        Thread.sleep(200);
        drive.setWobblePosPow(0,-.75); // arm is the power
        Thread.sleep(300); // milliseconds is the wait time
        drive.setWobblePosPow(0,0);
        Thread.sleep(300);
        drive.setWobblePosPow(-1,0);
        Thread.sleep(1000);
        drive.setWobblePosPow(1,0);
        drive.setWobblePosPow(0,.75); // arm is the power
        Thread.sleep(300); // milliseconds is the wait time
        drive.setWobblePosPow(0,0);
        Thread.sleep(300);
    }
}
