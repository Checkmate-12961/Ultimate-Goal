package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Disabled
@Autonomous
public class Snausage extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();

        drive.setWobblePosPow(0, 1);
        sleep(1000);
        drive.setWobblePosPow(-1,0);
        sleep(1000);
        drive.setWobblePosPow(0,-1);

        if(isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {

        }
    }
}
