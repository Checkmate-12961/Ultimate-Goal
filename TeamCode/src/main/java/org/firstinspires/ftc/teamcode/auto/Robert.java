package org.firstinspires.ftc.teamcode.auto;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DrunkenHippoDrive;

// Our glorious test auto mode
@Disabled
@SuppressWarnings("unused")
@Autonomous
public class Robert extends LinearOpMode {
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        DrunkenHippoDrive drive = new DrunkenHippoDrive(hardwareMap);

        waitForStart();

        depositWobble(drive);

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
        }
    }
    private void depositWobble(DrunkenHippoDrive drive){

    }
}
