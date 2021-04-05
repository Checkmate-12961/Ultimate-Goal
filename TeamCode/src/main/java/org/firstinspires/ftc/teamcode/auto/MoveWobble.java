package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.HungryHippoDrive;

@Autonomous
public class MoveWobble extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        HungryHippoDrive drive = new HungryHippoDrive(hardwareMap);

        waitForStart();

        depositWobble(drive);

        sleep(1500);

        collectWobble(drive);

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
        }
    }
    // Method to slap the wobble goal down that can be easily imported in other OpModes
    public static void depositWobble(HungryHippoDrive drive) throws InterruptedException {
        // Close the hand
        drive.setWobblePosPow(1,0);
        Thread.sleep(200);
        // Move out at -75% power
        drive.setWobblePosPow(0,-.75);
        Thread.sleep(300);
        // Stop moving the arm
        drive.setWobblePosPow(0,0);
        Thread.sleep(300);
        // Open the servo
        drive.setWobblePosPow(-1,0);
        Thread.sleep(1000);
        // Start closing the servo and move the arm up and away from the wobble
        drive.setWobblePosPow(1,.75);
        Thread.sleep(300);
        // Stop powering the arm and let its velocity carry it back
        drive.setWobblePosPow(0,0);
        Thread.sleep(300);
    }
    public static void collectWobble(HungryHippoDrive drive) throws InterruptedException {
        // Close hand
        drive.setWobblePosPow(1,0);
        Thread.sleep(200);
        // Move down and open hand
        drive.setWobblePosPow(-1, -.75);
        Thread.sleep(300);
        // Stop the arm
        drive.setWobblePosPow(0,0);
        Thread.sleep(200);
        // Close the servo
        drive.setWobblePosPow(1,0);
        Thread.sleep(1000);
        // Bring it in
        drive.setWobblePosPow(0,.6);
        Thread.sleep(600);
        // Stop powering the arm and let its velocity carry it back
        drive.setWobblePosPow(0,0);
        Thread.sleep(300);
    }
}
