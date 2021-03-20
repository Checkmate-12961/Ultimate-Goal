package org.firstinspires.ftc.teamcode.auto;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.HungryHippoDrive;

// Our glorious test auto mode
@SuppressWarnings("unused")
@Autonomous
public class Robert extends LinearOpMode {
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        HungryHippoDrive drive = new HungryHippoDrive(hardwareMap);

        waitForStart();

        depositWobble(drive);

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
        }
    }
    private void depositWobble(HungryHippoDrive drive){
        drive.setWobblePosPow(-1,0);
        sleep(200);
        drive.setWobblePosPow(0,-.75); // arm is the power
        sleep(300); // milliseconds is the wait time
        drive.setWobblePosPow(0,0);
        sleep(300);
        drive.setWobblePosPow(1,0);
        sleep(1000);
        drive.setWobblePosPow(-1,0);
        drive.setWobblePosPow(0,.75); // arm is the power
        sleep(300); // milliseconds is the wait time
        drive.setWobblePosPow(0,0);
        sleep(300);
    }
}
